-- Offline queue: a device that lost WiFi keeps its readings in flash and,
-- once back online, uploads them in batches with the time each was taken.
--
--   ingest_sensor_readings(p_token, p_readings) -> 'ok' | 'ok wifi_setup'
--
-- p_readings is a JSON array (at most 200 items) of
--   {"t": <unix seconds or null>, "do": .., "ph": .., "nh3": .., "fnh3": ..,
--    "wt": .., "at": .., "hum": ..}
-- Missing/null values are stored as null. "t" null means "now" (the device
-- didn't know the time yet).
--
-- Safe to resend: a reading already stored for the device at the same time is
-- skipped, so a batch whose reply was lost doesn't create duplicates.
-- Times in the future are treated as now; readings older than 60 days are
-- dropped (a wrong clock, not a real backlog).
--
-- Depends on 20261011130000_device_wifi_setup_request.sql (the WiFi setup
-- request is handed over here too). ingest_sensor_reading() stays as it is
-- for older firmware.

create or replace function public.ingest_sensor_readings(
  p_token text,
  p_readings jsonb
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

-- Devices call it with the anon key, like ingest_sensor_reading(); the
-- token/secret is what proves which device it is.
grant execute on function public.ingest_sensor_readings(text, jsonb) to anon, authenticated;
