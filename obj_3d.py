import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


obj_filename = "hj.obj"
obj_filename = "Squirtle.obj"
obj_filepath = 'obj_resource/'
# mtl_filename = ""

class OBJ:
    def __init__(self, filename, filepath):
        self.obj_filename = filename
        self.filepath = filepath
        self.mtl_filename = ''
        self.obj_load()
    
    # Load obj file
    def obj_load(self):
        v = []
        vt = []
        vn = []
        f = []

        # Read the obj file by line
        for line in open(self.filepath + self.obj_filename, "r"):
            values = line.split()
            if len(values) == 0: continue
            if values[0] == 'v':
                a=[ float(x) for x in values[1:]]
                v.append(a)
            elif values[0] == 'vt':
                a=[ float(x) for x in values[1:]]
                vt.append(a)
            elif values[0] == 'vn':
                a=[ float(x) for x in values[1:]]
                vn.append(a)
            elif values[0] == 'f':
                an = []
                for a in values[1:]:
                    a = a.split('//')
                    if len(a) == 1:
                        a = a[0].split('/')
                    a = [ int(x) - 1 for x in a]
                    an.append(a)
                f.append(an)
            elif values[0] == 'mtllib':
                mtl_filename = values[1]

        # for line in open(self.filepath + mtl_filename, "r"):
        #     print(line)

        # Transfrom v from a list to an array
        self.v = np.array(v).T
        # self.vt = np.array(vt).T
        # self.vn = np.array(vn).T

        # Normalize v
        scale = 0.2/self.v.max()
        self.v = self.v * scale

        # Calculate the model for plot
        self.model = [[], [], []]
        for i in range(len(f)):
            for j in range(3):
                self.model[j].append([])
            for j in f[i]:
                for k in range(3):
                    self.model[k][i].append(self.v[k][j[0]])

        # Recreate model for plotting
        self.model_plot = self.model
        for i in range(3):
            for m in self.model_plot[i]:
                m.append(m[0])

    def get_model(self, for_plot=True):
        if for_plot:
            return np.array(self.model_plot)
        else:
            return np.array(self.model)

    def obj_plot(self):
        fig = plt.figure()
        ax1 = plt.axes(projection='3d')
        plt.title('3D model frame')
        for i in range(len(self.model_plot[0])):
            ax1.plot3D(self.model_plot[0][i], self.model_plot[1][i], self.model_plot[2][i], 'b', linewidth=1)

        fig = plt.figure()
        ax = Axes3D(fig)
        plt.title('3D model colored on face')
        for i in range(len(self.model[0])):
            verts = [list(zip(self.model[0][i],self.model[1][i], self.model[2][i]))]
            ax.add_collection3d(Poly3DCollection(verts,facecolors='red',edgecolors='black'))

        fig = plt.figure()
        plt.title(' The front side of 3D model')
        for i in range(len(self.model_plot[0])):
            plt.fill(self.model[0][i], self.model[1][i], 'r')
            plt.plot(self.model_plot[0][i], self.model_plot[1][i], 'b', linewidth=1)
            
        plt.show()

if __name__ == '__main__':
    o = OBJ(filename=obj_filename, filepath=obj_filepath)
    o.obj_plot()