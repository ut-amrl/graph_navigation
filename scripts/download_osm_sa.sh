#!/bin/bash
wget https://download.geofabrik.de/south-america-latest.osm.pbf
mkdir osrm_south-america_cbf_mld
mv south-america-latest.osm.pbf osrm_south-america_cbf_mld
cd osrm_south-america_cbf_mld
# git clone git@github.com:fossgis-routing-server/cbf-routing-profiles.git
osrm-extract -p cbf-routing-profiles/foot.lua south-america-latest.osm.pbf
osrm-partition south-america-latest.osrm
osrm-customize south-america-latest.osrm