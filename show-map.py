import sys
import csv
from gmplot import gmplot

if (len(sys.argv) != 2):
    print ("Usage: %s filename" % sys.argv[0])
    sys.exit()

lats = []
lons = []
lats_tx = []
lons_tx = []

with open(sys.argv[1], newline='', encoding='utf-8-sig') as csvfile:
    csv_reader = csv.reader(csvfile, delimiter=',')
    for row in csv_reader:
         lats.append(float(row[0][:2] + "." + row[0][2:]))
         lons.append(float(row[1][:2] + "." + row[1][2:]))
         if (row[9] == "APRS UPDATE"):
             lats_tx.append(float(row[0][:2] + "." + row[0][2:]))
             lons_tx.append(float(row[1][:2] + "." + row[1][2:]))

gmap = gmplot.GoogleMapPlotter(lats[0], lons[0], 15)
gmap.plot(lats, lons, 'cornflowerblue', edge_width=1)
gmap.plot(lats_tx, lons_tx, 'red', edge_width=2.5)
gmap.scatter(lats_tx, lons_tx, 'r', marker=True)
gmap.draw("map.html")

print("Map generated succesfully")
