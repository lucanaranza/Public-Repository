import csv

input_file = "data_converted.csv"
output_file = "trajectory.kml"

coords = []

with open(input_file, newline="") as f:
    reader = csv.reader(f)

    next(reader)  # skip header row

    for row in reader:
        if len(row) < 5:
            continue

        try:
            altitude = float(row[2])
            lon = float(row[3])
            lat = float(row[4])

            coords.append(f"{lon},{lat},{altitude}")

        except:
            continue


kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Rocket Trajectory</name>

    <Placemark>
      <LineString>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>
          {" ".join(coords)}
        </coordinates>
      </LineString>
    </Placemark>

  </Document>
</kml>
"""

with open(output_file, "w") as f:
    f.write(kml)

print("KML created:", output_file)