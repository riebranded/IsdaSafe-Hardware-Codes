-- The app shows "Device connected to WiFi" once a device has joined the
-- network the owner chose (first setup, reconnecting, or changing networks).
-- For that, uploads now say which network the device is on:
--
--   devices.wifi_ssid            network name the device last uploaded from
--   devices.wifi_ssid_changed_at when that name last changed
--   devices.last_seen_at         last upload of any kind
--
-- ingest_sensor_readings() gets an optional p_wifi_ssid (firmware sends it);
-- older firmware, which doesn't, keeps working and just leaves wifi_ssid as
-- it was. ingest_sensor_reading() (single reading, older firmware) updates
-- last_seen_at too.

alter table public.devices add column if not exists wifi_ssid text;
alter table public.devices add column if not exists wifi_ssid_changed_at timestamptz;
alter table public.devices add column if not exists last_seen_at timestamptz;

-- Records an upload from v_device_id: last seen now, and the network if it
-- changed.
create or replace function public.note_device_upload(p_device_id text, p_wifi_ssid text)
returns void
language sql
security definer
set search_path to 'public'
as $$
  update devices
     set last_seen_at = now(),
         wifi_ssid = coalesce(nullif(left(p_wifi_ssid, 64), ''), wifi_ssid),
         wifi_ssid_changed_at = case
           when nullif(p_wifi_ssid, '') is not null
                and left(p_wifi_ssid, 64) is distinct from wifi_ssid
             then now()
           else wifi_ssid_changed_at
         end
   where id = p_device_id;
$$;

revoke execute on function public.note_device_upload(text, text) from public, anon, authenticated;

drop function if exists public.ingest_sensor_readings(text, jsonb);

create function public.ingest_sensor_readings(
  p_token text,
  p_readings jsonb,
  p_wifi_ssid text default null
)
returns text
language plpgsql
security definer
set search_path to 'public'
as $$
declare
  v_hash   text := encode(extensions.digest(coalesce(p_token, ''), 'sha256'), 'hex');
  v_device devices%rowtype;
  v_item   jsonb;
  v_time   timestamptz;
begin
  select * into v_device from devices where token_hash = v_hash;
  if not found then
    select d.* into v_device
      from devices d join device_credentials c on c.id = d.id
     where c.secret_hash = v_hash;
  end if;
  if not found then
    raise exception 'device not paired' using errcode = '28000';
  end if;

  if jsonb_typeof(p_readings) is distinct from 'array' then
    raise exception 'p_readings must be a JSON array' using errcode = '22023';
  end if;
  if jsonb_array_length(p_readings) > 200 then
    raise exception 'at most 200 readings per batch' using errcode = '22023';
  end if;

  perform note_device_upload(v_device.id, p_wifi_ssid);

  for v_item in select value from jsonb_array_elements(p_readings) loop
    v_time := case
      when jsonb_typeof(v_item -> 't') = 'number'
        then to_timestamp((v_item ->> 't')::double precision)
      else now()
    end;
    if v_time > now() then
      v_time := now();
    end if;
    continue when v_time < now() - interval '60 days';
    continue when exists (
      select 1 from sensor_readings
       where device_id = v_device.id and created_at = v_time
    );

    insert into sensor_readings (
      device_id, user_id, pond_id, created_at,
      dissolved_oxygen, ph, ammonia, free_ammonia,
      water_temperature, air_temperature, humidity
    ) values (
      v_device.id, v_device.user_id, v_device.pond_id, v_time,
      (v_item ->> 'do')::real, (v_item ->> 'ph')::real,
      (v_item ->> 'nh3')::real, (v_item ->> 'fnh3')::real,
      (v_item ->> 'wt')::real, (v_item ->> 'at')::real,
      (v_item ->> 'hum')::real
    );
  end loop;

  -- Same WiFi setup hand-over as ingest_sensor_reading().
  if v_device.wifi_setup_requested_at is not null then
    update devices set wifi_setup_requested_at = null where id = v_device.id;
    if v_device.wifi_setup_requested_at > now() - interval '30 minutes' then
      return 'ok wifi_setup';
    end if;
  end if;
  return 'ok';
end;
$$;

grant execute on function public.ingest_sensor_readings(text, jsonb, text) to anon, authenticated;

-- Older firmware's single-reading upload: mark it seen as well.
create or replace function public.ingest_sensor_reading(
  p_token text,
  p_dissolved_oxygen real default null,
  p_ph real default null,
  p_ammonia real default null,
  p_water_temperature real default null,
  p_air_temperature real default null,
  p_humidity real default null,
  p_free_ammonia real default null
)
returns text
language plpgsql
security definer
set search_path to 'public'
as $$
declare
  v_hash   text := encode(extensions.digest(coalesce(p_token, ''), 'sha256'), 'hex');
  v_device devices%rowtype;
begin
  select * into v_device from devices where token_hash = v_hash;
  if not found then
    select d.* into v_device
      from devices d join device_credentials c on c.id = d.id
     where c.secret_hash = v_hash;
  end if;
  if not found then
    raise exception 'device not paired' using errcode = '28000';
  end if;

  perform note_device_upload(v_device.id, null);

  insert into sensor_readings (
    device_id, user_id, pond_id,
    dissolved_oxygen, ph, ammonia, free_ammonia,
    water_temperature, air_temperature, humidity
  ) values (
    v_device.id, v_device.user_id, v_device.pond_id,
    p_dissolved_oxygen, p_ph, p_ammonia, p_free_ammonia,
    p_water_temperature, p_air_temperature, p_humidity
  );

  if v_device.wifi_setup_requested_at is not null then
    update devices set wifi_setup_requested_at = null where id = v_device.id;
    if v_device.wifi_setup_requested_at > now() - interval '30 minutes' then
      return 'ok wifi_setup';
    end if;
  end if;
  return 'ok';
end;
$$;
