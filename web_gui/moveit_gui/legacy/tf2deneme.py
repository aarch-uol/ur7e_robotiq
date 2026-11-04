
import rclpy 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from rclpy.node import Node
from rclpy.duration import Duration
class CoordinateListener(Node):

    def __init__(self):

        super().__init__('CoordinateListener')

        self.tf_buffer = Buffer(cache_time=Duration(seconds = 10.))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.get_transform)
        self.trans = None

        while rclpy.ok() and self.trans == None:
            rclpy.spin(self)

        self.destroy_node()

    def get_transform(self):
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'tool0', now) #timeout=Duration(seconds=1.0)
            self.trans = trans
            print(trans.transform.translation.x)
            print(trans.transform.translation.y)
            print(trans.transform.translation.z)
            #print(self.msg.transform.translation)
            #self.timer.cancel()
        except TransformException as e:
            print('could not find the transformation: ' + str(e))

def main():
    rclpy.init()
    coordinate = CoordinateListener()


if __name__ == '__main__':
    main()