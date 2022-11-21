from PIL import Image
import numpy as np
from collections import defaultdict


def is_valid(value):
    return value != 0


class Png2PlyConverter():
    '''Read Trispector PNG file and save it as PLY file'''

    def __init__(self, source_file, target_file):
        self.source_file = source_file
        self.target_file = target_file

    def extract_data(self):
        # Load png and metadata
        source = Image.open(self.source_file)
        source.load()  # Needed for .png EXIF data
        metadata = source.info  # 'Dx', 'Dy', 'Dz', 'X0', 'Y0','Z0', 'CameraZ'
        dx, self.dy, dz = float(metadata['Dx']), float(metadata['Dy']), float(metadata['Dz'])
        x0, z0 = float(metadata['X0']), float(metadata['Z0'])

        # Get intensity
        intensity = source.crop((0, 0, source.size[0], source.size[1] / 3))  # Top 1/3 of image
        intensity = np.asarray(intensity).flatten()

        # Get heightmap
        heightmap = source.crop((0, source.size[1] / 3, source.size[0], source.size[1]))  # Bottom 2/3 of image
        width = heightmap.size[0]
        height = heightmap.size[1] // 2
        heightmap = np.asarray(heightmap).flatten()

        # Extract height data and x-pos
        self.z_data = np.zeros(len(heightmap) // 2)
        self.x_data = np.zeros(len(heightmap) // 2)
        for i in range(0, len(heightmap), 2):
            z = heightmap[i] + heightmap[i + 1] * 256
            self.z_data[i // 2] = z0 + z * dz
            self.x_data[i // 2] = x0 + ((i // 2) % width) * dx

        self.z_data = self.z_data.reshape((height, width))
        self.x_data = self.x_data.reshape((height, width))
        self.intensity_data = intensity.reshape((height, width))

    def write_ply_header(self, file, num_vertices, num_triangles, save_color, write_triangles):
        file.write("ply \n")
        file.write("format ascii 1.0 \n")
        file.write(f"element vertex {num_vertices} \n")
        file.write("property float x \n")
        file.write("property float y \n")
        file.write("property float z \n")
        if (save_color):
            file.write("property uchar red \n")
            file.write("property uchar green \n")
            file.write("property uchar blue \n")
        else:
            file.write("property float Intensity \n")
        if write_triangles:
            file.write(f"element face {num_triangles} \n")
            file.write("property list uchar int vertex_index \n")
        file.write("end_header \n")

    def write_ply(self, save_color, write_triangles):

        vertex_dict = defaultdict(dict)
        vertices = []
        triangles = []
        vertex_number = 0
        num_triangles = 0
        height, width = self.z_data.shape

        # Format vertices.
        it = np.nditer([self.z_data, self.x_data, self.intensity_data], flags=['multi_index'])
        while not it.finished:  # Multi-iterators slightly more performant.
            ix = it.multi_index[0]
            iy = it.multi_index[1]
            y = it.multi_index[0] * self.dy
            z = it[0]
            x = it[1]
            intensity = it[2]
            it.iternext()
            if (is_valid(z)):
                if (save_color):
                    # %formatting
                    vertices.append(f"{x} {y} {z} {intensity} {intensity} {intensity}\n")
                else:
                    c = intensity / 255.0
                    # %formatting
                    vertices.append(f"{x} {y} {z} {c}\n")
                vertex_dict[ix][iy] = vertex_number
                vertex_number += 1

        # Note: Triangulation does some things "incorrectly" (skipping edges, and making missing data holes too big)
        if (write_triangles):
            for j in range(height):
                for i in range(width):
                    if is_valid(self.z_data[j, i]):
                        if (j != (height - 1) and i != (width - 1)):  # Point is candidate for triangle
                            if (is_valid(self.z_data[j, i + 1]) and is_valid(self.z_data[j + 1, i])):
                                triangles.append(
                                    f"3 {vertex_dict[j][i]} {vertex_dict[j][i + 1]} {vertex_dict[j + 1][i]} \n")
                                num_triangles += 1
                        if (j != 0 and i != 0):
                            if (is_valid(self.z_data[j, i - 1]) and is_valid(self.z_data[j - 1, i])):
                                triangles.append(
                                    f"3 {vertex_dict[j][i]} {vertex_dict[j][i - 1]} {vertex_dict[j - 1][i]} \n")
                                num_triangles += 1

        # Write to file
        f = open(self.target_file, 'w')
        num_vertices = np.count_nonzero(self.z_data != 0)
        self.write_ply_header(f, num_vertices, num_triangles, save_color, write_triangles)
        f.write(''.join(vertices))  # write all points to the file
        f.write(''.join(triangles))  # write all triangles to the file
