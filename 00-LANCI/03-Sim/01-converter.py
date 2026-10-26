import pandas as pd
import math

# === 1. User Input: Launch site GPS coordinates ===
launch_lat_deg = 39.394361
launch_lon_deg = -8.292278



# === 2. Earth radius for WGS-84 ellipsoid ===
EARTH_RADIUS = 6378137  # meters

# === 3. Load your data ===
df = pd.read_csv("data.csv")
df.columns = ["time", "altitude", "east", "north"]

# === 4. Convert ENU (East, North) to lat/lon ===
def enu_to_latlon(north, east, base_lat, base_lon):
    d_lat = north / EARTH_RADIUS
    d_lon = east / (EARTH_RADIUS * math.cos(math.radians(base_lat)))
    lat = base_lat + math.degrees(d_lat)
    lon = base_lon + math.degrees(d_lon)
    return lat, lon

# === 5. Apply the conversion to each row ===
df[['latitude', 'longitude']] = df.apply(
    lambda row: pd.Series(enu_to_latlon(row['north'], row['east'], launch_lat_deg, launch_lon_deg)),
    axis=1
)

# === 6. Reorder and save output ===
df[['time', 'altitude', 'longitude', 'latitude']].to_csv("data_converted.csv")


#39°23'39.7"N 8°17'32.2"W

#39.394361, -8.292278