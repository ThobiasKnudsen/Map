import os 
import rasterio
from rasterio.warp import transform
import sys
import numpy as np
from numba import jit

def _read_geotiff(file_path):
    @jit(nopython=True)
    def inside_func(array):
        for i in range(array.shape[0]):
            for j in range(array.shape[1]):
                if array[i, j] < 0:
                    array[i, j] = 0
        return array
    with rasterio.open(file_path) as dataset:
        # Read the first (and possibly only) band of the raster
        array = dataset.read(1)
        array = inside_func(array)
    return array
class map_metadata:
    def __init__(self, file_path, max_mountain_height=1024):
        if file_path[-4:] == ".tif":
            with rasterio.open(file_path) as dataset:
                self.byte_width = dataset.width
                self.byte_height = dataset.height
                self.max_mountain_height = max_mountain_height
                corners = [(0, 0), (self.byte_width, 0), (0, self.byte_height), (self.byte_width, self.byte_height)]

                for i in range(len(corners)):
                    x_geo, y_geo = dataset.transform * corners[i]
                    lon, lat = rasterio.warp.transform(dataset.crs, 'EPSG:4326', [x_geo], [y_geo])
                    if i == 0:
                        self.lat1 = lat[0]
                        self.lon1 = lon[0]
                    elif i == 1:
                        self.lat2 = lat[0]
                        self.lon2 = lon[0]
                    elif i == 2:
                        self.lat3 = lat[0]
                        self.lon3 = lon[0]
                    elif i == 3:
                        self.lat4 = lat[0]
                        self.lon4 = lon[0]
                
                        
        elif file_path[-12:] == "metadata.txt":
            with open(file_path, 'r') as file:
                text = file.read()
            lines = text.split('\n')
            for i in range(len(lines)):
                key, value = lines[i].split('=')
                if 'byte width' in key:
                    self.byte_width = float(value)
                if 'byte height' in key:
                    self.byte_height = float(value)
                if 'lat and lon' in key:
                    lat, lon = value.split(',')
                    if i == 2:
                        self.lat1, self.lon1 = float(lat), float(lon)
                    elif i == 3:
                        self.lat2, self.lon2 = float(lat), float(lon)
                    elif i == 4:
                        self.lat3, self.lon3 = float(lat), float(lon)
                    elif i == 5:
                        self.lat4, self.lon4 = float(lat), float(lon)

    def print_self(self):
        print(f"width={self.byte_width} | height={self.byte_height}")
        print(f"lat and lon for top right={self.lat1},{self.lon1}\n")
        print(f"lat and lon for top left={self.lat2},{self.lon2}\n")
        print(f"lat and lon for bottom right={self.lat3},{self.lon3}\n")
        print(f"lat and lon for bottom left={self.lat4},{self.lon4}\n")

    def is_this_equal_to_other(self, other_rect):
        if self.byte_width != other_rect.byte_width:
            return False
        elif self.byte_height != other_rect.byte_height:
            return False
        elif self.lat1 != other_rect.lat1:
            return False
        elif self.lon1 != other_rect.lon1:
            return False
        elif self.lat2 != other_rect.lat2:
            return False
        elif self.lon2 != other_rect.lon2:
            return False
        elif self.lat3 != other_rect.lat3:
            return False
        elif self.lon3 != other_rect.lon3:
            return False
        elif self.lat4 != other_rect.lat4:
            return False
        elif self.lon4 != other_rect.lon4:
            return False
        return True
    
    def write_to_file(self, file_path):
        if os.path.basename(file_path)[-4:] != ".txt":
            print("WARNING | did not write to file {file_path} because its not .txt file")
            return
        
        string=f"byte width={self.byte_width}\n"
        string+=f"byte height={self.byte_height}\n"
        string+=f"max mountain height={self.max_mountain_height}\n"
        string+=f"lat and lon for top right={self.lat1},{self.lon1}\n"
        string+=f"lat and lon for top left={self.lat2},{self.lon2}\n"
        string+=f"lat and lon for bottom right={self.lat3},{self.lon3}\n"
        string+=f"lat and lon for bottom left={self.lat4},{self.lon4}\n"
        
        
        with open(file_path,'w') as file:
            file.write(string)

def make_bin_files_from_DTM_and_DOM_files(folder_path_DTM, folder_path_DOM, write_to_folder_path, max_mountain_height = 1024):
    # max_mountain_height is used to know how much any height can be multiplied
    # by so that its under 65536 aka sizeof(uint16) so that the heighest presition
    # is stored. in the default case every float32 is multiplied by 64 or 65536/1024
    # before its converted into uint16

    if not os.path.exists(folder_path_DTM):
        print("given DTM folder path doesnt exist. terminating")
        return
    if not os.path.exists(folder_path_DOM):
        print("given DOM folder path doesnt exist. terminating")
        return

    bin_folder = write_to_folder_path+"\\bin"
    if not os.path.exists(bin_folder):
        os.makedirs(bin_folder)
        print(f"Folder created {bin_folder}")
    else:
        print(f"Folder exists {bin_folder}")

    files_DTM = [f for f in os.listdir(folder_path_DTM) if f.endswith('.tif')]
    files_DOM = [f for f in os.listdir(folder_path_DOM) if f.endswith('.tif')]
    for i in range(len(files_DTM)):
        files_DTM[i] = os.path.basename(files_DTM[i])
        files_DOM[i] = os.path.basename(files_DOM[i])

    common_files = list(set(files_DTM).intersection(set(files_DOM)))

    for i in range(len(common_files)):
        rect_DTM = map_metadata(folder_path_DTM+f"\\{common_files[i]}")
        rect_DOM = map_metadata(folder_path_DOM+f"\\{common_files[i]}")
        if not rect_DTM.is_this_equal_to_other(rect_DOM):
            print(f"WARNING | these two tif files has not the same metadata:")
            print(f"        | {folder_path_DTM}\\{common_files[i]}:")
            rect_DTM.print_self()
            print(f"        | {folder_path_DOM}\\{common_files[i]}:")
            rect_DOM.print_self()
            continue

        array2D_DTM = _read_geotiff(folder_path_DTM+f"\\{common_files[i]}")
        array2D_DOM = _read_geotiff(folder_path_DOM+f"\\{common_files[i]}")
        if (len(array2D_DTM) != len(array2D_DOM) or len(array2D_DTM[0]) != len(array2D_DOM[0])):
            print(f"WARNING | these two tif files has not the same metadata:")
            print(f"        | {folder_path_DTM}\\{common_files[i]}: {len(array2D_DTM)}X{len(array2D_DTM[0])}")
            print(f"        | {folder_path_DOM}\\{common_files[i]}: {len(array2D_DOM)}X{len(array2D_DOM[0])}")
            continue

        max_height = None
        pixel_width_meters = None
        with rasterio.open(folder_path_DTM+f"\\{common_files[i]}") as dataset:
            band1 = dataset.read(1)
            max_height = np.max(band1)

            n=1
            while max_height>n:
                n*=2
            max_height = n

            transform = dataset.transform
            pixel_width_meters = transform[0]
        
        print(max_height)
        print(pixel_width_meters)

        # writong metadata
        name_without_extension = os.path.splitext(common_files[i])[0]
        metadata_file_path = bin_folder+"\\"+name_without_extension+"_metadata.txt"
        rect_DTM.write_to_file(metadata_file_path)
        print("made and wrote to: "+bin_folder+"\\"+name_without_extension+"_metadata.txt")

        #writing data
        array2D_delta_tarrain = (array2D_DOM-array2D_DTM).astype(np.uint8)
        array2D_DTM_ui16 = (array2D_DTM * (65536/max_height)).astype(np.uint16)

        with open(bin_folder+"\\"+name_without_extension+"_DTM.bin",'wb') as file:
            array2D_DTM_ui16.tofile(file)
        print(f"made and wrote to: {bin_folder}\\{name_without_extension}_DTM.bin")

        with open(bin_folder+"\\"+name_without_extension+"_DOMminusDTM.bin",'wb') as file:
            array2D_delta_tarrain.tofile(file)
        print(f"made and wrote to: {bin_folder}\\{name_without_extension}_DOMminusDTM.bin")

def main():
    if len(sys.argv) != 4:
        print("makeBinFromGeoTIFF.exe takes three arguments:")
        print("1. folder path to DTM GeoTIFF files")
        print("2. folder path to DOM GeoTIFF files")
        print("3. folder path to where you want the files to be stored")
        sys.exit(1)
    make_bin_files_from_DTM_and_DOM_files(sys.argv[1], sys.argv[2], sys.argv[3])

if __name__ == "__main__":
    main()