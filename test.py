# try to import pydrake, if error, print error message
try:
    import pydrake
    print("pydrake imported successfully")

except ImportError as e:
    print("Error importing pydrake: ", e)