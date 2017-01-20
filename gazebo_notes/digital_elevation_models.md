# Digital Elevation Models

**Digital Elevation Model** (DEM) is a 3D representation of a terrain's surface that does not inlcude any objects like buildings or vegetation.

- are frequently created by using a combination of sensors, such as LIDAR, radar, or cameras
- can be represented as a grid of elevations (raster) or as a vector-based triangular irregular network (TIN)
    - currently Gazebo only supports raster data in the supported formats available in [GDAL][2]

The terrain elevations for ground positions are sampled at regularly-spaced horizontal intervals.

Main motivation to support DEMs is to simulate a realistic terrain.  Rescue or agriculture applications might be interested in testing their robot behaviors using a simulated terrain that matches the real world.

For more info on [DEM][1]

## Bring DEM support to Gazebo

To use DEM files install GDAL libraries.

```
$ sudo apt-get install gdal-bin libgdal-dev libgdal1h python-gdal
```

## DEM file and the definition into SDF format

For acurate simulation, you will need elevation data. You can get this from several organizations.

As an example, we can download DEM file of Mount St. Helens [before][3] and [after][4]. These files are in public domain & are distributed by [USGS][5]

Unzip the file and rename it `mtsthelens.dem`:
    
    ```
    cd ~/Downloads
    wget https://bitbucket.org/osrf/gazebo_tutorials/raw/default/dem/files/mtsthelens_before.zip
    unzip ~/Downloads/mtsthelens_before.zip -d /tmp
    mv /tmp/30.1.1.1282760.dem /tmp/mtsthelens.dem
    ```

DEM files have big resolutions & Gazebo cannot handle it, so it is best to adjust resolution of your DEM. 

- following command will scale terrain to 129x129 & will copy into the Gazebo `media/dem/` directory
    
    ```
    $ mkdir -p /tmp/media/dem/
    $ gdalwarp -ts 129 129 /tmp/mtsthelens.dem /tmp/media/dem/mtsthelens_129.dem
    ```

DEM file in Gazebo is loaded similar to a heightmap image.  Gazebo automatically detects if the file is a plain image or a DEM file.

- Create file `volcano.world` & copy the next content. Save file anywhere you want. (example: `/tmp`) 

    ```
    
    ```

**Return to [Categories: Build a World][6]**

[1]: http://en.wikipedia.org/wiki/Digital_elevation_model
[2]: http://www.gdal.org/
[3]: https://bitbucket.org/osrf/gazebo_tutorials/raw/default/dem/files/mtsthelens_before.zip
[4]: https://bitbucket.org/osrf/gazebo_tutorials/raw/default/dem/files/mtsthelens_after.zip
[5]: http://nationalmap.gov/elevation.html
[6]: ../gazebo_categories/build_world.md 
