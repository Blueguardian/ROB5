import os
import lxml.etree
import lxml.builder
import numpy as np


class XMLHandler():
    __FILENAME = "points.xml"
    __PATH = os.path.join(os.curdir, __FILENAME)
    __BUILDER = lxml.etree


    def __init__(self):
        with open(self.__PATH, 'w+') as f:
            f.close()

    def clearfile(self):
        with open(self.__PATH, 'w+') as f:
            f.close()

    def writepoints(self, cloud):
        ROOT = self.__BUILDER.Element('Pointcloud')
        i = 0
        for numPoints in np.ndindex(cloud.shape[0]):
            self.__BUILDER.SubElement(ROOT, f"aXPoint",  index=f"{i}").text = f"{cloud[numPoints, 0][0]}"
            i += 1
        i = 0
        for numPoints in np.ndindex(cloud.shape[0]):
            self.__BUILDER.SubElement(ROOT, f"aYPoint", index=f"{i}").text = f"{cloud[numPoints, 1][0]}"
            i += 1
        tree = lxml.etree.ElementTree(ROOT)
        tree.write(self.__PATH, xml_declaration=True, encoding="UTF-8", pretty_print=True)


