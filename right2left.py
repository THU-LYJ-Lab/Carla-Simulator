# Usage: right2left input.obj [output.obj]
# [IMPORTANT] SHOULD BE USED ON CARLA AXIS SYSTEM
# This will read from input.obj (right-handed axis system) then convert it into output.obj (left-handed axis system)
# output.obj is optional, if left empty, this will output the result into output.obj

import os
import sys
import numpy as np

def main():

    args = list(sys.argv)
    if len(args) > 3 or len(args) < 2:
        print("Usage: right2left input.obj [output.obj]")
        print("[IMPORTANT] SHOULD BE USED ON CARLA AXIS SYSTEM")
        print("This will read from input.obj (right-handed axis system) then convert it into output.obj (left-handed axis system)")
        print("output.obj is optional, if left empty, this will output the result into output.obj")
        return
    elif len(args) == 2:
        args.append("output.obj")

    obj_lines = []
    # Load the Object
    with open(args[1],'r',encoding='utf-8') as f:
        obj_lines = f.readlines()

    # Convert it into carla axis system
    with open(args[2],'w',encoding='utf-8') as w:
        for line in obj_lines:
            if line.startswith('v '):
                new_point = np.array(line.split()[1:], dtype=np.float64)
                w.write(' '.join(['v', str(new_point[1]), str(new_point[0]), str(new_point[2])]) + '\n')
            elif line.startswith('vt '):
                w.write(line)
            elif line.startswith('vn '):
                new_point = np.array(line.split()[1:], dtype=np.float64)
                w.write(' '.join(['vn', str(new_point[1]), str(new_point[0]), str(new_point[2])]) + '\n')
            elif line.startswith('f '):
                face = line.split()
                for i in range(1, 4):
                    face[i] = face[i].split('/')
                    face[i] = [str(int(x)) for x in face[i]]
                w.write(' '.join(['f', '/'.join(face[1]), '/'.join(face[2]), '/'.join(face[3])]) + '\n')
            else:
                w.write(line)

if __name__ == '__main__':
    main()
