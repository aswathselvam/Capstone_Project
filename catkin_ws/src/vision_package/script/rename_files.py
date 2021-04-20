#ffmpeg -i s8.mp4 -vf fps=3 Label_%d.jpg

import os
path = '/home/aswath/Videos/myvideo/s8'
for filename in os.listdir(path):
    prefix, num = filename[:-4].split('_')
    print(prefix," ",num)
    num = num.zfill(4)
    new_filename = prefix + "_" + str(num) + ".jpg"
    os.rename(os.path.join(path, filename), os.path.join(path, new_filename))
