# Mars terrain source

The local heightmap is a compact crop derived from the USGS Astrogeology Science Center's **Mars MRO HiRISE Hadriacus Palus DEM 1m** product:

https://astrogeology.usgs.gov/search/map/mars_mro_hirise_hadriacus_palus_dem_1m_and_orthophoto_50cm_mosaics

Source data is released for use without constraints; retain the USGS/NASA citation when redistributing the simulator. The runtime never accesses the source URL: the checked-in `.bytes` height stream and generated PBR maps are local assets.
