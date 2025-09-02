import time, sys, argparse
from rclpy.time import Time
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy import init, ok, spin_once, shutdown

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("source")                 
    ap.add_argument("target")                 
    args = ap.parse_args()
    
    init(args=None)
    try:
        node=Node("wait_tf")
        buf=Buffer() 
        tl=TransformListener(buf, node)
        end=time.time()+60
        found=False
        while ok() and time.time()<end:
            spin_once(node, timeout_sec=0.1)
            if buf.can_transform(args.source, args.target, Time()):
                found=True 
                break
            time.sleep(0.1)
    # swallow Ctrl-C
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        shutdown()
        sys.exit(0 if found else 1)
        
if __name__ == '__main__':
    main()