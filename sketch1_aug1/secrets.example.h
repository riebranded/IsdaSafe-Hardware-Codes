// Template for secrets.h. Copy this file to secrets.h (same folder) and
// fill in the real values; secrets.h is git-ignored.
#pragma once

// Optional default network, used only until one is chosen on the device's
// setup hotspot or by the website over USB (see isdasafe_wifi.h). Leave the
// placeholders to start with no network.
#define WIFI_SSID         "YOUR_WIFI_SSID"
#define WIFI_PASSWORD     "YOUR_WIFI_PASSWORD"

// Same value as SUPABASE_ANON_KEY in the app's .env.
#define SUPABASE_ANON_KEY "PASTE_SUPABASE_ANON_KEY_HERE"
