import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Quaternion, Pose, Twist, Point
from gazebo_msgs.srv import DeleteModel, SpawnModel
import numpy as np

def eval_table(table):
    raise NotImplementedError('Heuristic function for organized.')

class TableManager(object):
    def __init__(self, table_model='simple_table'):
        rospy.init_node('table_manager')
        self.models = None
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

        x0 = 0.55 + 0.1875
        y0 = 0.0 + 0.1875
        grid_size = 0.125
        self._grid_poses = []
        for i in range(4):
            self._grid_poses.append([])
            x = x0 - float(i) * grid_size
            for j in range(4):
                y = y0 - float(j) * grid_size
                pose = Pose(Point(x,y,0.4), Quaternion(0,0,0,1))
                self._grid_poses[-1].append(pose)

    def _gazebo_callback(self, msg):
        models = {}
        for name, pose in zip(msg.name, msg.pose):
            models[name] = pose
        self.models = models

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
        rospy.loginfo('Move {} to ({}, {})'.format(state.model_name, x, y))
        self._pub.publish(state)

    def spawn(self):
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        source='/home/yan/roganized_ws/src/ROganized/roganized_gazebo/models/simple_cube/model.sdf'
        with open(source, "r") as f:
            cube_xml = f.read()

        for i in range(4):
            cube_pose = self._grid_poses[0][i%4]
            x = cube_pose.position.x
            y = cube_pose.position.y
            model_name = 'cube_'+str(i)
            rospy.loginfo('Spawn '+model_name+' at ({},{})'.format(x,y))
            spawn_model(model_name, cube_xml, "", cube_pose, "world")
