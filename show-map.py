import csv
from gmplot import gmplot

lats = []
lons = []
lats_tx = []
lons_tx = []

with open('points.csv', newline='', encoding='utf-8-sig') as csvfile:
    csv_reader = csv.reader(csvfile, delimiter=';')
    for row in csv_reader:
         lats.append(float(row[0][:2] + "." + row[0][2:]))
         lons.append(float(row[1][:2] + "." + row[1][2:]))
         if (row[11] == "true"):
             lats_tx.append(float(row[0][:2] + "." + row[0][2:]))
             lons_tx.append(float(row[1][:2] + "." + row[1][2:]))

gmap = gmplot.GoogleMapPlotter(lats[0], lons[0], 15)
gmap.plot(lats, lons, 'cornflowerblue', edge_width=2)
gmap.scatter(lats_tx, lons_tx, 'r', marker=True)
gmap.draw("map.html")
