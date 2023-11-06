import cv2  # Import the OpenCV library
import numpy as np
import time
import sys
import threading
import framebuffer
from pkg_resources import parse_version


gstreamerCameraFound = False
piCameraFound = False
try:
    import picamera
    from picamera.array import PiRGBArray
    piCameraFound = True
except ImportError:
    print("Camera.py: picamera module not available - using OpenCV interface instead")

def isRunningOnArlo():
    """Return True if we are running on Arlo, otherwise False."""
    return piCameraFound or gstreamerCameraFound

# Black magic in order to handle differences in namespace names in OpenCV 2.4 and 3.0
OPCV3 = parse_version(cv2.__version__) >= parse_version('3')

def capPropId(prop):
    """returns OpenCV VideoCapture property id given, e.g., "FPS
       This is needed because of differences in the Python interface in OpenCV 2.4 and 3.0
    """
    return getattr(cv2 if OPCV3 else cv2.cv, ("" if OPCV3 else "CV_") + "CAP_PROP_" + prop)


def gstreamer_pipeline(capture_width=1280, capture_height=720, framerate=30):
    """Utility function for setting parameters for the gstreamer camera pipeline"""
    return (
        "libcamerasrc !"
        "videobox autocrop=true !"
        "video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "videoconvert ! "
        "appsink"
        % (
            capture_width,
            capture_height,
            framerate,
        )
    )

class CaptureThread(threading.Thread):
    """Internal worker thread that captures frames from the camera"""
    
    def __init__(self, cam, framebuffer):
        threading.Thread.__init__(self)
        self.cam = cam
        self.framebuffer = framebuffer
        self.terminateThreadEvent = threading.Event()


    def run(self):
        while not self.terminateThreadEvent.is_set():
            if piCameraFound:
                # Use piCamera

                #self.cam.capture(self.rawCapture, format="bgr", use_video_port=True)
                #image = self.rawCapture.array
            
                # clear the stream in preparation for the next frame
                #self.rawCapture.truncate(0)
                
                if sys.version_info[0] > 2:
                    # Python 3.x
                    image = np.empty((self.cam.resolution[1], self.cam.resolution[0], 3), dtype=np.uint8)
                else:
                    # Python 2.x
                    image = np.empty((self.cam.resolution[1] * self.cam.resolution[0] * 3,), dtype=np.uint8)
                    
                self.cam.capture(image, format="bgr", use_video_port=True)
                
                if sys.version_info[0] < 3:
                    # Python 2.x
                    image = image.reshape((self.cam.resolution[1], self.cam.resolution[0], 3))

            else:  # Use OpenCV
                retval, image = self.cam.read()  # Read frame

                if not retval:  # Error
                    print("CaptureThread: Could not read next frame")
                    exit(-1)

            # Update framebuffer
            self.framebuffer.new_frame(image)
        
        
    def stop(self):
        """Terminate the worker thread"""
        self.terminateThreadEvent.set()


class Camera(object):
    """This class is responsible for doing the image processing. It detects known landmarks and 
    measures distances and orientations to these."""
    
    def __init__(self, camidx, robottype='arlo', useCaptureThread = False):
        """Constructor:
             camidx - index of camera
             robottype - specify which robot you are using in order to use the correct camera calibration. 
                         Supported types: arlo, frindo, scribbler, macbookpro"""

        self.useCaptureThread = useCaptureThread

        # Set camera calibration info
        if robottype == 'arlo':
            self.imageSize = (1280, 720)
            self.intrinsic_matrix = np.asarray([500, 0., self.imageSize[0] / 2.0, 0.,
                   500, self.imageSize[1] / 2.0, 0., 0., 1.], dtype = np.float64)
            self.intrinsic_matrix.shape = (3, 3)
            self.distortion_coeffs = np.asarray([0., 0., 2.0546093607192093e-02, -3.5538453075048249e-03, 0.], dtype = np.float64)

        elif robottype == 'macbookpro':
            self.imageSize = (1280, 720)
            self.intrinsic_matrix = np.asarray([9.4328095162920715e+02, 0., self.imageSize[0] / 2.0, 0.,
                   9.4946668595979077e+02, self.imageSize[1] / 2.0, 0., 0., 1.], dtype = np.float64)
            self.intrinsic_matrix.shape = (3, 3)
            self.distortion_coeffs = np.asarray([0., 0., -1.6169374082976234e-02, 8.7657653170062459e-03, 0.], dtype = np.float64)
        
        else:
            print("Camera.__init__: Unknown robot type")
            exit(-1)
            

        # Open a camera device for capturing                 
        if piCameraFound:
            # piCamera is available so we use this
            #self.cam = picamera.PiCamera(camidx)
            self.cam = picamera.PiCamera(camera_num=camidx, resolution=self.imageSize, framerate=30)
            
            if not self.useCaptureThread:
                self.rawCapture = PiRGBArray(self.cam, size=self.cam.resolution)
                
            time.sleep(2) # wait for the camera            
            
            # Too slow code - instead just use cam.capture
            #self.capture_generator = self.cam.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)

            gain = self.cam.awb_gains
            self.cam.awb_mode='off'
            self.cam.awb_gains = gain
            
            self.cam.shutter_speed = self.cam.exposure_speed
            self.cam.exposure_mode = 'off'


            print("shutter_speed = ", self.cam.shutter_speed)
            print("awb_gains = ", self.cam.awb_gains)
            
            
            print("Camera width = ", self.cam.resolution[0])
            print("Camera height = ", self.cam.resolution[1])
            print("Camera FPS = ", self.cam.framerate)


        else:  # Use OpenCV interface

            # We next try the gstreamer interface
            self.cam = cv2.VideoCapture(gstreamer_pipeline(
                capture_width=self.imageSize[0], capture_height=self.imageSize[1]),
                apiPreference=cv2.CAP_GSTREAMER)
            if not self.cam.isOpened(): # Did not work

                # We try first the generic auto-detect interface
                self.cam = cv2.VideoCapture(camidx)
                if not self.cam.isOpened():  # Error
                    print("Camera.__init__: Could not open camera")
                    exit(-1)
                else:
                    print("Camera.__init__: Using OpenCV with auto-detect interface")
            else:
                gstreamerCameraFound = True
                print("Camera.__init__: Using OpenCV with gstreamer")

            time.sleep(1) # wait for camera
            
            # Set camera properties
            self.cam.set(capPropId("FRAME_WIDTH"), self.imageSize[0])
            self.cam.set(capPropId("FRAME_HEIGHT"), self.imageSize[1])
            #self.cam.set(capPropId("BUFFERSIZE"), 1) # Does not work
            #self.cam.set(capPropId("FPS"), 15)
            self.cam.set(capPropId("FPS"), 30)
        
            time.sleep(1)
        
            # Get camera properties
            print("Camera width = ", int(self.cam.get(capPropId("FRAME_WIDTH"))))
            print("Camera height = ", int(self.cam.get(capPropId("FRAME_HEIGHT"))))
            print("Camera FPS = ", int(self.cam.get(capPropId("FPS"))))
        
        # Initialize chessboard pattern parameters
        self.patternFound = False
        self.patternSize = (3,4)
        self.patternUnit = 50.0 # mm (size of one checker square)
        self.corners = []

        # Initialize aruco detector
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        # Set the correct physical marker size here
        self.arucoMarkerLength = 0.15  # [m] actual size of aruco markers (in object coordinate system)
        
        # Initialize worker thread and framebuffer
        if self.useCaptureThread:
            print("Using capture thread")
            self.framebuffer = framebuffer.FrameBuffer((self.imageSize[1], self.imageSize[0], 3))
            self.capturethread = CaptureThread(self.cam, self.framebuffer)
            self.capturethread.start()
            time.sleep(0.75)

    def __del__(self):
        if piCameraFound:
            self.cam.close()
        
    def terminateCaptureThread(self):
        if self.useCaptureThread:
            self.capturethread.stop()
            self.capturethread.join()


    def get_next_frame(self):
        """Gets the next available image frame from the camera."""
        if self.useCaptureThread:
            img = self.framebuffer.get_frame()
            
            if img is None:
                img = np.array((self.imageSize[0], self.imageSize[1], 3), dtype=np.uint8)
                        
        else:
            if piCameraFound:
                # Use piCamera
            
                self.cam.capture(self.rawCapture, format="bgr", use_video_port=True)
                img = self.rawCapture.array
            
                # clear the stream in preparation for the next frame
                self.rawCapture.truncate(0)
            
            else: # Use OpenCV
                retval, img = self.cam.read()  # Read frame

                if not retval:  # Error
                    print("Camera.get_colour: Could not read next frame")
                    exit(-1)
        
        return img


    # ArUco object detector
    def detect_aruco_objects(self, img):
        """Detect objects in the form of a binary ArUco code and return object IDs, distances (in cm) and
        angles (in radians) to detected ArUco codes. The angle is computed as the signed angle between
        translation vector to detected object projected onto the x-z plabe and the z-axis (pointing out
        of the camera). This corresponds to that the angle is measuring location along the horizontal x-axis.

        If no object is detected, the returned variables are set to None."""
        self.aruco_corners, self.ids, rejectedImgPoints = cv2.aruco.detectMarkers(img, self.arucoDict)
        self.rvecs, self.tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(self.aruco_corners, self.arucoMarkerLength, self.intrinsic_matrix, self.distortion_coeffs)


        if not isinstance(self.ids, type(None)):

            # for i in range(self.rvecs.shape[1]):

            #     # get rotation on y-axis from the rotation vector
            #     _, ry, _ = self.rvecs[0, i]

            #     # Define a vector with length 20, angle 45, and then rotate it further by the y-axis rotation:
            #     translation_vector = np.array([[np.cos(np.pi*0.25-ry), 0, np.sin(np.pi*0.25-ry)]]) * .20

            #     # Extend each tvec by the translation_vector
            #     self.tvecs[0, i] += translation_vector.ravel()

            dists = np.linalg.norm(self.tvecs, axis=len(self.tvecs.shape) - 1) * 100
            # Make sure we always return properly shaped arrays
            dists = dists.reshape((dists.shape[0],))
            ids = self.ids.reshape((self.ids.shape[0],))

            # Compute angles
            angles = np.zeros(dists.shape, dtype=dists.dtype)
            for i in range(dists.shape[0]):
                tobj = self.tvecs[i] * 100 / dists[i]
                zaxis = np.zeros(tobj.shape, dtype=tobj.dtype)
                zaxis[0,-1] = 1
                xaxis = np.zeros(tobj.shape, dtype=tobj.dtype)
                xaxis[0,0] = 1

                # We want the horizontal angle so project tobjt onto the x-z plane
                tobj_xz = tobj
                tobj_xz[0,1] = 0
                # Should the sign be clockwise or counter-clockwise (left or right)?
                # In this version it is positive to the left as seen from the camera.
                direction = -1*np.sign(tobj_xz[0,0])  # The same as np.sign(np.dot(tobj, xaxis.T))
                angles[i] = direction * np.arccos(np.dot(tobj_xz, zaxis.T))
        else:
            dists = None
            ids = None
            angles = None
        return ids, dists, angles


    def draw_aruco_objects(self, img):
        """Draws detected objects and their orientations on the image given in img."""
        if not isinstance(self.ids, type(None)):
            outimg = cv2.aruco.drawDetectedMarkers(img, self.aruco_corners, self.ids)
            for i in range(self.ids.shape[0]):
                outimg = cv2.drawFrameAxes(outimg, self.intrinsic_matrix, self.distortion_coeffs,
                                           self.rvecs[i], self.tvecs[i], self.arucoMarkerLength)
        else:
            outimg = img

        return outimg

if (__name__=='__main__'):
    print("Opening and initializing camera")
    
    cam = Camera(0, 'macbookpro', useCaptureThread = True)
    #cam = Camera(0, 'macbookpro', useCaptureThread = False)
    #cam = Camera(0, 'arlo', useCaptureThread = True)
    
    # Open a window
    WIN_RF1 = "Camera view"
    cv2.namedWindow(WIN_RF1)
    cv2.moveWindow(WIN_RF1, 50, 50)
        
    #WIN_RF3 = "Camera view - gray"
    #cv2.namedWindow(WIN_RF3)
    #cv2.moveWindow(WIN_RF3, 550, 50)
    
    while True:
        
        action = cv2.waitKey(10)
        
        if action == ord('q'):  # Quit
            break
    
        # Fetch next frame
        #colour = cam.get_colour()
        colour = cam.get_next_frame()
                
        # Convert to gray scale
        #gray = cv2.cvtColor(colour, cv2.COLOR_BGR2GRAY )
        #loggray = cv2.log(gray + 1.0)
        #cv2.normalize(loggray, loggray, 0, 255, cv2.NORM_MINMAX)
        #gray = cv2.convertScaleAbs(loggray)
        
        # Detect objects
        objectType, distance, angle = cam.detect_aruco_objects(colour)
        if objectType != 'none':
            print("Object type = ", objectType, ", distance = ", distance, ", angle = ", angle, ", colourProb = ", colourProb)

        # Draw detected pattern
        # cam.draw_object(colour)
        cam.draw_aruco_objects(colour)
        IDs, dists, angles = cam.detect_aruco_objects(colour)
        if not isinstance(IDs, type(None)):
            for i in range(len(IDs)):
                print("Object ID = ", IDs[i], ", Distance = ", dists[i], ", angles = ", angles[i])

        # Draw detected objects
        cam.draw_aruco_objects(colour)

    
        # Show frames
        cv2.imshow(WIN_RF1, colour)
        
        # Show frames
        #cv2.imshow(WIN_RF3, gray)
        
        
    # Close all windows
    cv2.destroyAllWindows()

    # Clean-up capture thread
    cam.terminateCaptureThread()

