# -*- coding: utf-8 -*-
"""
Created on Tue Dec 18 16:00:53 2018

@author: HPC
"""

import netCDF4
import numpy as np
from osgeo import gdal
from osgeo import osr

#Reading in data from files and extracting said data

#%%
source='A:/WRFPackage/wrfnetcdf/2012/4/wrfout_d03_2012-04-01.nc'
ncfile = netCDF4.Dataset(source, 'r') 
dataw=ncfile.variables['T2'][44,:,:]
lon=ncfile.variables['XLONG'][1,:,:]
lat=ncfile.variables['XLAT'][1,:,:]
#%%

#  Initialize the Image Size
image_size = np.shape(dataw)

#%%




# set geotransform
nx = image_size[1]
ny = image_size[0]
xmin=np.min(lon)
ymin=np.min(lat)
xmax=np.max(lon)
ymax =np.max(lat)
xres = (xmax - xmin) / float(nx)
yres = (ymax - ymin) / float(ny)
geotransform = (xmin, xres, 0, ymin, 0, yres)

# create the 3-band raster file
dst_ds = gdal.GetDriverByName('GTiff').Create('myGeoTIFF.tif', nx, ny, 1, gdal.GDT_Float32)

dst_ds.SetGeoTransform(geotransform)    # specify coords
srs = osr.SpatialReference()            # establish encoding
srs.ImportFromEPSG(4326)                # WGS84 lat/long
dst_ds.SetProjection(srs.ExportToWkt()) # export coords to file
dst_ds.GetRasterBand(1).WriteArray(dataw)   # write r-band to the raster
   # write b-band to the raster
dst_ds.FlushCache()                     # write to disk
dst_ds = None                        # save, close



#%%

import matplotlib.pyplot as plt

#%%


plt.contourf(lon,lat,dataw)
plt.colorbar()