**Vehicle Detection Project**

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Apply a color transform and append binned color features, as well as 
histograms of color, to your HOG feature vector. 
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.


## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
###Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
###Writeup / README

####1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Vehicle-Detection/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

###Histogram of Oriented Gradients (HOG)

####1. Explain how (and identify where in your code) you extracted HOG features from the training images.

The code for this step is contained in the first code cell of the IPython 
notebook in `extract_features()`.  

I explored various color spaces and found three color channels that helped 
identify different parts of the car. These channels were as follows:

1. Lightness Channel in HLS color space, which accentuated the differences 
between the car and the road:

![L Channel]("output_images/L Channel.png")

2. Saturation Channel in HLS color space, which typically had elevated 
levels on all different colors of car:

![S Channel]("output_images/S Channel.png")

3. A Channel in LAB color space, which easily identified the cars' tail lights:

![A Channel]("output_images/A Channel.png")

After finding these color spaces, I created color histograms, HOG features, 
and spacial features to feed into the model.


####2. Explain how you settled on your final choice of HOG parameters.

I started with the default parameters in the Udacity code and tweaked them 
to see how performance would change. Surprisingly, of the combinations I 
tested, the original ones resulted in the best performance.

####3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

First (code block 5), I used sklearn's SelectKBest to identify features that 
had a pvalue 
of less than 0.05 when compared with the data. I then used it to only 
include these features from then on.

Then, I trained a Random Forest Classifier on the data. I chose Random 
Forest because in testing, it was comparable to an SVM in accuracy but also 
supports probability estimates, which I wanted to leverage to get a quick 
estimate of the confidence for each prediction. I use this later on in my 
heatmaps.

###Sliding Window Search

####1. Describe how (and identify where in your code) you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?

I used the code from the lessons (`sliding_window()`) to generate each 
sliding window. I decided on the scales mainly by checking how the heatmap 
responded with different scales. I found that the smaller scales I used 
(48x48 and 64x64) accentuated the true positives while the larger scale 
(96x96) helped offset any false positives.

####2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

I optimized the classifier by using GridSearchCV to tune its parameters 
and testing out different feature combinations until I arrived at 
something I liked. Here's an example of an image from my pipeline:

![Example Output]("output_images/example_output.png")

Far from perfect, but if smoothing is used between frames this is good enough 
for most of a video feed.

### Video Implementation

####1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
Here's a [link to my video result](./project_video_output.mp4)


####2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

For each sliding window, got the probability estimate from the classifier 
and added this into a heatmap array. At the end, I divided each pixel value 
by the number of times it appeared in a sliding window (some, especially 
those on the edge of the image, appeared less than others and so were 
artificially low)
 to arrive at
 an 
average probability for each pixel. I then thresholded this heatmap.
  I then 
used 
`scipy.ndimage.measurements.label()`
 to 
identify individual 
blobs in the heatmap.  I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.  


---

###Discussion

####1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

My pipeline could definitely improve. It currently does a good job 
identifying the positions of the cars, but not necessarily their accurate 
boundaries. The windows sometimes don't cover the entire car. During the 
beginning of the video feed, there are sometimes more false positives as 
smoothing has not fully taken affect.
 To improve 
these issues, I would spend some more time improving my classifier and maybe 
also 
developing some new sliding windows.

Another major issue is the speed; the code is exceedingly slow. One way to 
fix this is to decrease the number of estimators in the Random Forest; this,
 however, decreases performance. If I had more time, I might try 
 experimenting with trying to get confidence/probability estimates for SVMs 
 to speed up this process.
