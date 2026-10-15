-- Store un-ionized (free) ammonia NH3 alongside total ammonia nitrogen.
--   ammonia       = TAN, mg/L, as measured by the SEN0711
--   free_ammonia  = free NH3, mg/L, computed on the device from TAN, pH and
--                   water temperature (Emerson et al. 1975)
-- The new parameter defaults to null, so devices running older firmware keep
-- uploading unchanged (their rows just have free_ammonia = null).

alter table public.sensor_readings add column if not exists free_ammonia real;

comment on column public.sensor_readings.ammonia is
  'Total ammonia nitrogen (TAN, NH3-N + NH4+-N), mg/L, measured by the SEN0711';
comment on column public.sensor_readings.free_ammonia is
  'Un-ionized ammonia NH3, mg/L, computed from TAN, pH and water temperature (Emerson 1975)';

drop function if exists public.ingest_sensor_reading(text, real, real, real, real, real, real);

create function public.ingest_sensor_reading(
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
as $function$
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
  return 'ok';
end;
$function$;

grant execute on function public.ingest_sensor_reading(text, real, real, real, real, real, real, real)
  to anon, authenticated, service_role;
