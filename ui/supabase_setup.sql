-- Run once in the Supabase SQL editor. Safe to re-run.
-- DS18B20 temperature string readings for the dashboard's Temp Profile view; columns mirror sensor_data.db.
-- Like measurements and weather, packet_id is unique: every base station uploads what it hears and
-- receiver.py upserts on packet_id, so a packet heard by several receivers is stored once.
create table if not exists public.temp_string (
    id bigint generated always as identity primary key,
    timestamp timestamptz not null,
    node_id bigint not null,
    seq integer,
    simulated boolean not null default false,
    probes integer,
    valid integer,
    latitude double precision,
    longitude double precision,
    packet_id bigint,
    received_at timestamptz,
    rx_snr real,
    rx_rssi integer,
    hops integer,
    relay_node bigint,
    gateway_id bigint,
    t1 real, t2 real, t3 real, t4 real, t5 real, t6 real, t7 real,
    t8 real, t9 real, t10 real, t11 real, t12 real, t13 real, t14 real
);

create unique index if not exists temp_string_packet on public.temp_string (packet_id);
create index if not exists temp_string_timestamp on public.temp_string (timestamp desc);

-- The dashboard reads with the publishable key; only the receiver's secret key can write.
alter table public.temp_string enable row level security;
drop policy if exists "dashboard reads temp_string" on public.temp_string;
create policy "dashboard reads temp_string" on public.temp_string for select to anon, authenticated using (true);
