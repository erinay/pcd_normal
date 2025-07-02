import imageio
import os

# Directory containing your PNG images
image_folder = 'data/NoMotion/occup_figures/' 
# Output video file name
output_video = 'occup_movi_NoMotion.mp4'

# Get a sorted list of image file paths
image_files = sorted([os.path.join(image_folder, f) for f in os.listdir(image_folder) if f.endswith('.png')])

# Create the video
with imageio.get_writer(output_video, mode='I', fps=10) as writer: # fps can be adjusted
    for filename in image_files:
        image = imageio.imread(filename)
        writer.append_data(image)
