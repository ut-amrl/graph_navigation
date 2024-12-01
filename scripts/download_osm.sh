#!;bin/bash
wget https://download.geofabrik.de/north-america/us/texas-latest.osm.pbf
mkdir osrm_texas_cbf_mld
mv texas-latest.osm.pbf osrm_texas_cbf_mld
cd osrm_texas_cbf_mld
git clone git@github.com:fossgis-routing-server/cbf-routing-profiles.git
osrm-extract -p cbf-routing-profiles/foot.lua texas-latest.osm.pbf
osrm-partition texas-latest.osrm
osrm-customize texas-latest.osrm