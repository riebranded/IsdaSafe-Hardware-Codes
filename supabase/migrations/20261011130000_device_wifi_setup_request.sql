-- Lets the owner change an ONLINE device's WiFi from the website without
-- touching the device: the website flags the device, and the next upload's
-- reply tells it to open its setup hotspot ("IsdaSafe-XXXX"), where the new
-- network is chosen from a phone. No WiFi password ever reaches the database.
--
-- ingest_sensor_reading() keeps its signature and still returns text: 'ok',
-- or 'ok wifi_setup' when a request is pending. Older firmware ignores the
-- reply body, so it keeps working.

alter table public.devices add column if not exists wifi_setup_requested_at timestamptz;

comment on column public.devices.wifi_setup_requested_at is
  'Owner asked (from the website) for the device to open its WiFi setup hotspot; cleared when the device picks it up';

-- Owner-only: flags the device. Returns false if it isn't the caller's.
create or replace function public.request_device_wifi_setup(p_device_id text)
returns boolean
language plpgsql
security definer
set search_path to 'public'
as $$
begin
  update devices
     set wifi_setup_requested_at = now()
   where id = p_device_id
     and user_id = auth.uid();
  return found;
end;
$$;

revoke execute on function public.request_device_wifi_setup(text) from public, anon;
grant execute on function public.request_device_wifi_setup(text) to authenticated;

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

  insert into sensor_readings (
    device_id, user_id, pond_id,
    dissolved_oxygen, ph, ammonia, free_ammonia,
    water_temperature, air_temperature, humidity
  ) values (
    v_device.id, v_device.user_id, v_device.pond_id,
    p_dissolved_oxygen, p_ph, p_ammonia, p_free_ammonia,
    p_water_temperature, p_air_temperature, p_humidity
  );

  -- Hand a pending WiFi setup request to the device, once. Requests older
  -- than 30 minutes are dropped: nobody is standing by the device any more.
  if v_device.wifi_setup_requested_at is not null then
    update devices set wifi_setup_requested_at = null where id = v_device.id;
    if v_device.wifi_setup_requested_at > now() - interval '30 minutes' then
      return 'ok wifi_setup';
    end if;
  end if;
  return 'ok';
end;
$$;
