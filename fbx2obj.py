# Usage: fbx2obj input.fbx [output.obj]
# This will read from input.fbx then convert it into output.obj
# output.obj is optional, if left empty, this will output the result into output.obj

import os
import sys
import fbx

def main():

    args = list(sys.argv)
    if len(args) > 3 or len(args) < 2:
        print("Usage: fbx2obj input.fbx [output.obj]")
        print("This will read from input.fbx then convert it into output.obj")
        print("output.obj is optional, if left empty, this will output the result into output.obj")
        return
    elif len(args) == 2:
        args.append("output.obj")

    # Create an SDK manager                                                                                           
    manager = fbx.FbxManager.Create()

    # Create a scene
    scene = fbx.FbxScene.Create(manager, "")

    # Create an importer object                                                                                                  
    importer = fbx.FbxImporter.Create(manager, "")

    # Path to the .obj file
    milfalcon = args[1]

    # Specify the path and name of the file to be imported                                                                            
    importstat = importer.Initialize(milfalcon, -1)

    importstat = importer.Import(scene)

    # Create an exporter object                                                                                                  
    exporter = fbx.FbxExporter.Create(manager, "")

    save_path = args[2]

    # Specify the path and name of the file to be imported                                                                            
    exportstat = exporter.Initialize(save_path, -1)

    exportstat = exporter.Export(scene)

if __name__ == '__main__':
    main()
