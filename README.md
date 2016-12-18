# CS 455 Term Project

## Requirements

Must have OpenCV 3+ and g++ version with at least C++11 support.

## How to Run

Compile the program first by running 'make',
Then run './gui <0,1> <path-to-image>' where 0 indicates that
the image should be opened as a grayscale and 1 indicates that the image
should be opened in color (RGB). For point generation and triangulation, please
open in the image in grayscale (option 0).

The following are useful/relevant commands:

ESC .......... quit the program
SPACE ........ reset the image
e ............ export the image
r ............ generate random points on an image from its edges
s ............ generate random points on an image with Shi-Tomasi detection
t ............ triangulate points on image (only run after running 'r' or 's')
1 ............ triangulate points on an image from random points of its edges
2 ............ triangulate points on an image from Shi-Tomasi detection points

Note: '1' and '2' are just 'r' and 's' combined with 't'. The difference is that
'1' and '2' do not produce circles generated from 'r' and 's'.

## Corner Detection

To calculate strong seed points for our triangulations, we propose the technique of corner detection.

### Motivations

### The Procedure

We will utilize Harris-Stephens and Shi-Tomasi corner detection.

## Delaunay Triangulation

We use radial sweep as the algorithm of choice for generating Delaunay triangulations.

## Compression

### Using Triangulation in Compression

We explore the means for which we can use triangulation in compression. Compression serves to minimize storage requirements for data. However, we usually must sacrifice image quality in exchange for space savings. From some standpoints, we care more about the image quality than we do its storage requirements. If we're taking the consumer's perspective, for instance, then we're sharing photos with family and friends through methods such as email, cloud storage hosts (e.g. Dropbox, Google Drive), and social media (e.g.Facebook, Instagram). We should not easily notice that the image has changed after compression. As such, we ask ourselves what we hope to achieve by maintaining high image quality. For purposes other than sharing, we may instead aim to communicate across the subject of the image. This may prove useful for applications such as neural networks where we do not care for the finer details, but the overall subejct of the image. At a large scale, we may be transmitting gigabytes or terabytes of visual data. When sent uncompressed, these images can slow down analysis time and increase our bandwidth expenses. We'd like to use compression, especially in environments where we have low bandwidth, unstable connectivity, or data caps (e.g. on specific ISPs or via mobile carriers).

### The Procedure

Unlike existing triangulation algorithms, we want to break down our images to consistently sized triangular subregions and apply recoloring. By doing so, we're cutting down the color or intensity information needed to store the image from a single pixel to a subregion of pixels. We can go as far as to create a new file format, or simply rely on the changes of recoloring the image. The compression code hosted on GitHub will explore space savings for both. The user selects a size for which to draw triangles. This size will increase until the subject can no longer be determined without prior context. The size for which the user last recognized the subject will be used to draw the trianglular subregions. The average intensity or color is chosen to fill the region and then we save the new image. That alone can result in some savings, but for lower quality or blurred images, may not be sufficient. As such, we will also consider a new file format (.cs455) for which we store only vital information to enable reconstructing the image at later times. Instead of storing all pixel information, we only store information to allow us to construct each subregion. As per our results, this can cut our storage requirements by over 90%. The grayscale examples, for instance, have seen 98% on average in savings.

### Future Applications

[x] Compare compression results between grayscale and color.
[x] Test larger images (MB)
[ ] Test even larger images (GB)
[ ] How can we make this procedure more efficient?
[ ] We're consistent with our triangle sizes - can we adjust based on the image's colors and intensities?
[ ] Can we obscure the image with triangulation to allow for protecting the image contents and only allow for the intended receiving party to unlock it (i.e. restore the image)?
