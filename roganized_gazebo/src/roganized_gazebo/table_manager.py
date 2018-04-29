import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Quaternion, Pose, Twist, Point
from gazebo_msgs.srv import DeleteModel, SpawnModel
import os

def _l1(pose1, pose2):
    return abs(pose1.position.x - pose2.position.x)+\
           abs(pose1.position.y - pose2.position.y)

def _adjacent(pose1, pose2):
    if abs(pose1.position.x - pose2.position.x) < 0.105 or\
       abs(pose1.position.y - pose2.position.y) < 0.105:
        return 1.0
    return 0.0

class TableManager(object):
    def __init__(self, table_model='simple_table', size=0.5, grids=5):
        rospy.init_node('table_manager')
        self.models = {}
        self.cubes = []
        self._sub = rospy.Subscriber('/gazebo/model_states',ModelStates,\
                                     self._gazebo_callback)
        self._pub = rospy.Publisher('/gazebo/set_model_state', ModelState,\
                                    queue_size=10)
        rospy.loginfo('Getting gazebo objects...')
        while not rospy.is_shutdown() and self.models is None:
            rospy.sleep(0.1)

        for name, _ in self.models.iteritems():
            rospy.loginfo('Import model '+name)
        #x0 = 0.55 + 0.1875
        #y0 = 0.0 + 0.1875
        x0 = 0.55 + float(size)/2.0*(float(grids)-1.0)/float(grids)
        y0 = 0.0 + float(size)/2.0*(float(grids)-1.0)/float(grids)
        grid_size = float(size)/float(grids)
        self._grid_poses = []
        for i in range(grids):
            self._grid_poses.append([])
            x = x0 - float(i) * grid_size
            for j in range(grids):
                y = y0 - float(j) * grid_size
                pose = Pose(Point(x,y,0.36), Quaternion(0,0,0,1))
                self._grid_poses[-1].append(pose)

    def _gazebo_callback(self, msg):
        models = {}
        for name, pose in zip(msg.name, msg.pose):
            #if name == 'cube_3':
            #    print 'boom'
            #models[name] = pose
            self.models[name] = pose
        #self.models = models

    def clear(self):
        rospy.wait_for_service("gazebo/delete_model") 
        delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        for name, _ in self.models.iteritems():
            if 'cube' in name:
                delete_model(name)

    def move_cube(self, id, i, j):
        state = ModelState()
        state.model_name = 'cube_{}'.format(id)
        state.pose = self._grid_poses[i][j]
        x = state.pose.position.x
        y = state.pose.position.y
        #rospy.loginfo('Move {} to ({}, {})'.format(state.model_name, x, y))
        self._pub.publish(state)

    def spawn(self):
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        # source='/home/yan/roganized_ws/src/ROganized/roganized_gazebo/models/simple_cube/model.sdf'

        source='/home/robert/roganized_ws/src/ROganized/roganized_gazebo/models/simple_cube/model.sdf'
        with open(source, "r") as f:
            cube_xml = f.read()

        for i in range(4):
            cube_pose = self._grid_poses[0][i%4]
            x = cube_pose.position.x
            y = cube_pose.position.y
            model_name = 'cube_'+str(i)
            rospy.loginfo('Spawn '+model_name+' at ({},{})'.format(x,y))
            spawn_model(model_name, cube_xml, "", cube_pose, "world")
        

    def score(self):
        score = 0.0
        for pair in [('cube_0','cube_1'),('cube_0','cube_2'),\
                     ('cube_0','cube_3'),('cube_1','cube_2'),\
                     ('cube_1','cube_3'),('cube_2','cube_3')]:
            score -= 1.2 * _l1(self.models[pair[0]], self.models[pair[1]])
            score += _adjacent(self.models[pair[0]], self.models[pair[1]])
        return score
