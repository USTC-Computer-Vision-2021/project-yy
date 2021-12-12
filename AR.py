from ARutils import *
from obj_3d import OBJ

# demo
# file_name0 = 'box.png'
# file_name1 = 'box_in_scene.png'

# instance
# file_name0 = 'book.jpg'
# file_name1 = 'book_in_scene.jpg'

file_name0 = '5G.jpg'
file_name1 = '5G_com.jpg'
img_file_path = 'img_resource/'

# 3D model
obj_file_name = "Squirtle.obj"
obj_file_path = 'obj_resource/'



class AR():
    def __init__(self, file_name0, file_name1, img_file_path='', obj_file_name='', obj_file_path=''):
        '''
        file_name0:     Reference image
        file_name1:     Object image
        img_file_path:  The path of image
        obj_file_name:  3D model, only 'obj' file is accepted
        obj_file_path:  The path of obj file
        '''
        file_source0 = img_file_path + file_name0
        file_source1 = img_file_path + file_name1

        '''
        Calculating the SIFT algorithm will take lots of time.
        So when the first time, we calculate the keypoints and
        descriptors of the image and save the data. If we need 
        to load the same image, we only need to load the data
        but no need to calculate the image again.
        '''

        l0, d0 = load_data(file_source0)
        l1, d1 = load_data(file_source1)
        
        #  Match the features and calculate the homography matrix
        matches = sift.match_twosided(d0, d1)
        ndx = matches.nonzero()[0]
        fp = homography.make_homog(l0[ndx, :2].T)
        ndx2 = [int(matches[i]) for i in ndx]
        tp = homography.make_homog(l1[ndx2, :2].T)

        model = homography.RansacModel()
        H, inliers = homography.H_from_ransac(fp, tp, model)

        # Calculate the camera calibration matrix K
        sift_width = max(l0[ndx,0]) - min(l0[ndx,0])
        sift_height = max(l0[ndx,1]) - min(l0[ndx,1])

        length = min(sift_width, sift_height)
        center = [min(l0[ndx,0]) + sift_width/2, min(l0[ndx,1]) + sift_height/2]
        # corner = [center[0] - length/2, center[0] + length/2, center[1] - length/2, center[1] + length/2] 

        K = my_calibration(center, length)
        

        # Calculate the coordinate points of a 3D cube
        box = cube_points([0, 0, 0.1], 0.1)

        # Calculate the first camera matrix
        cam1 = camera.Camera(hstack((K, dot(K, array([[0], [0], [-1]])))))

        # Get the bottom of the cube and project it onto the feature point of the first picture
        self.box_cam1 = cam1.project(homography.make_homog(box[:, :5]))

        # Using the homography matrix to covert box into the second picture
        box_trans = homography.normalize(dot(H, self.box_cam1))

        # Calculate the second camera matrix from cam1 and K
        cam2 = camera.Camera(dot(H, cam1.P))
        A = dot(linalg.inv(K), cam2.P[:, :3])
        A = array([A[:, 0], A[:, 1], cross(A[:, 0], A[:, 1])]).T
        cam2.P[:, :3] = dot(K, A)

        # Get the cube and project it onto the feature point of the second picture
        self.box_cam2 = cam2.project(homography.make_homog(box))

        # load the obj model and project it onto the second picture
        if obj_file_name != '':
            obj = OBJ(obj_file_name, obj_file_path).get_model()
            
            self.obj_model = []
            for i in range(len(obj[0])):
                self.obj_model.append(cam2.project(homography.make_homog(np.array([obj[0][i], obj[1][i], obj[2][i]]))))

        # Get the height and width of the second picture
        im1 = array(Image.open(file_source1))
        self.height, self.width = int(im1.shape[0]), int(im1.shape[1])

        # Get the transition from the center of the picture to the center of
        self.center_trans = [(box_trans[0][0] + box_trans[0][2] + box_trans[0][1] + box_trans[0][3])/4, 
        (box_trans[1][0] + box_trans[1][2] + box_trans[1][1] + box_trans[1][3])/4]

        # Data instantiation
        self.file_name0, self.file_name1 = file_name0, file_name1
        self.file_source0, self.file_source1 = file_source0, file_source1
        self.img_file_path, self.obj_path = img_file_path, obj_file_path

        self.l0, self.d0 = l0, d0
        self.l1, self.d1 = l1, d1

        self.ndx = ndx
        self.ndx2 = ndx2
        self.K = K
        self.cam2 = cam2
        self.box_trans = box_trans

    def Frame_Model(self, if_obj_model=True):
        '''
        This function will plot the square on the first and 
        second picture. If want to plot 3D model on the 
        picture, input the name of the obj model when 
        instantiating the AR class, or a cube will be plot 
        on the picture.
        '''
        if if_obj_model:
            obj_model = self.obj_model
        else:
            obj_model = None

        plot_projection(self.file_source0, self.file_source1, self.box_cam1, self.box_trans, self.box_cam2, 
        self.ndx, self.ndx2, self.l0, self.l1, obj_model)


    def Solid_3D_Model(self):
        '''
        This function will plot a red teapot on the second 
        picture.
        '''

        # Initiating the pygame window
        pygame.init()
        pygame.display.set_mode((self.width, self.height), OPENGL | DOUBLEBUF)
        pygame.display.set_caption("AR Teapot")

        # Calculate the matrix of modelview from camera
        Rt = dot(linalg.inv(self.K), self.cam2.P)

        draw_background(self.file_source1)

        set_projection_from_camera(self.K, self.height, self.width)
        set_modelview_from_camera(Rt, self.height, self.width, self.center_trans)
        # set_modelview_from_camera(cam2.P)

        # Draw the red teapot, the parameter is the scale of the teapot
        draw_teapot(0.05)
        pygame.display.flip()


        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()
                if event.type == MOUSEBUTTONDOWN:
                    pressed_array = pygame.mouse.get_pressed()
                    if pressed_array[0]: # the left button is pressed
                        pos = pygame.mouse.get_pos()  # get the position of mouse
                        print(pos)

if __name__ == '__main__':
    ar = AR(file_name0, file_name1, img_file_path, obj_file_name, obj_file_path)
    ar.Frame_Model()
    ar.Solid_3D_Model()