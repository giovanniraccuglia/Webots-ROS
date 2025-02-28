import rospy
from pynput.keyboard import Key, Listener, KeyCode
from geometry_msgs.msg import Twist
import tiago_pkg.utils as Utils

rospy.init_node("Joystick", anonymous=True)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
Utils.call_service('wheel_left_joint', 'set_position', float('inf'))
Utils.call_service('wheel_right_joint', 'set_position', float('inf'))

def on_press(key):
    msg = Twist()
    if key == KeyCode.from_char('w'):
        msg.linear.x = 0.5
        msg.angular.z = 0
    elif key == KeyCode.from_char('s'):
        msg.linear.x = -0.5
        msg.angular.z = 0
    elif key == KeyCode.from_char('a'):
        msg.linear.x = 0
        msg.angular.z = 0.05
    elif key == KeyCode.from_char('d'):
        msg.linear.x = 0
        msg.angular.z = -0.05
    pub.publish(msg)
    
# Setta la velocità a 0 se nessun tasto viene premuto o se si premono tasti a caso
def on_release(key):
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)
    if key == Key.esc:
        print("Termino il programma!")
        return False

# Inizializzo il listener che si occuperà di ascoltare la pressione dei tasti
with Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()
