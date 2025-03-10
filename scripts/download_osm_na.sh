#!/bin/bash
# wget https://download.geofabrik.de/north-america-latest.osm.pbf
# mkdir osrm_north-america_cbf_mld
mv north-america-latest.osm.pbf osrm_north-america_cbf_mld
cd osrm_north-america_cbf_mld
# git clone git@github.com:fossgis-routing-server/cbf-routing-profiles.git
osrm-extract -p cbf-routing-profiles/foot.lua north-america-latest.osm.pbf
osrm-partition north-america-latest.osrm
osrm-customize north-america-latest.osrm