import ctypes
import glob
import os

class LibraryLoadError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

def __load_library(name):
        libdir = os.path.dirname(__file__)
        if libdir == "":
            libdir = "./"
        files = glob.glob(libdir + "/lib/*" + name + "*")

        library = None
        for f in files:
            try:
                library = ctypes.cdll.LoadLibrary(f)
                break
            except:
                continue

        if not library:
            raise LibraryLoadError("Unable to load library: " + name)

        return library

def load_library():
    try:
        global pyfluid
        pyfluid = __load_library("pyfluid")
    except:
        pass

def unload_library():
    try:
        global pyfluid
        ctypes.windll.kernel32.FreeLibrary(pyfluid._handle)
    except:
        pass


pyfluid = __load_library("pyfluid")