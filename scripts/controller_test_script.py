import rospy
from bravo_controllers.srv import EvalMinErrorMove, EvalPoint2Point, EvalPoint2PointRequest, EvalTrajectory, EvalTrajectoryRequest
import os

def checkMateFolder(folder):
    if not os.path.exists(folder):
        os.mkdir(folder)

if __name__ == "__main__":
    rospy.init_node("controller_test")
    
    folder = "testing"
    reps = 10
    full = True
    take_rev = True

    # get services for each type
    print("Waiting for services")
    if full:
        rospy.wait_for_service("/bravo/controller_eval/eval_min_error_move")
        minSRV = rospy.ServiceProxy("/bravo/controller_eval/eval_min_error_move", EvalMinErrorMove)
        print("Got min error service")
        rospy.wait_for_service("/bravo/controller_eval/eval_pt_2_pt")
        p2pSRV = rospy.ServiceProxy("/bravo/controller_eval/eval_pt_2_pt", EvalPoint2Point)
        print("Got point 2 point service")
    rospy.wait_for_service("/bravo/controller_eval/eval_traj")
    trjSRV = rospy.ServiceProxy("/bravo/controller_eval/eval_traj", EvalTrajectory)
    print("Got Trajectory Service")
    # make folder to save this result set
    folder_dir = "/home/hunter/catkin_ws/" + folder
    if not os.path.exists(folder_dir):
        os.mkdir(folder_dir)

    trajReq = EvalTrajectoryRequest()
    trajReq.n_reps = 100
    trajReq.traj_n = reps
    trajReq.traj_rev = take_rev
    trajReq.dt = 0.1

    if full:
        # check min error
        
        #res = minSRV(0.005, 0.1, "min_error")
        #print("Min Error for 5mm motion in 0.1 seconds")
        #print(res.message)

        
        # point 2 point
        print("Point to Point Motion")
        p2pReq = EvalPoint2PointRequest()
        p2pReq.a = [0.337, -0.15, 0.2]
        p2pReq.b = [0.45, 0.15, 0.12]
        p2pReq.n_reps = reps
        p2pReq.exp_name = "/p2p/results"
        checkMateFolder(folder_dir + "/p2p/")
        res = p2pSRV(p2pReq)
        print(res.message)

        
        #assert 1 == 0 
        # xy circle
        #trajReq.traj_type = "oval"
        #trajReq.exp_name = folder + "/xy_oval/"
        #trajReq.dx = 0.145
        #trajReq.dy = 0.15
        #trajReq.dz = 0.005
        #xyRes = trjSRV(trajReq)
        #print(xyRes.message)
        

        # xz cirlce
        print("XZ Oval trajectory tracking")
        checkMateFolder(folder_dir + "/xz_oval/")
        trajReq.traj_type = "oval"
        trajReq.exp_name = "/xz_oval/results"
        trajReq.dx = 0.10
        trajReq.dy = 0.15
        trajReq.dz = 0.2
        xzRes = trjSRV(trajReq)
        print(xzRes.message)
        
        # xy square
        print("XY Square Trajectory Tracking")
        checkMateFolder(folder_dir + "/xy_square/")
        trajReq.traj_type = "square"
        trajReq.exp_name = "/xy_square/results"
        trajReq.sx = 0.18
        trajReq.sy = 0.225
        trajReq.sz = 0.02
        xySqRes = trjSRV(trajReq)
        print(xySqRes.message)
        
        # xz square
        print("XZ Square Trajectory Tracking")
        checkMateFolder(folder_dir + "/xz_square/")
        trajReq.traj_type = "square"
        trajReq.exp_name = "/xz_square/results"
        trajReq.sx = 0.16
        trajReq.sy = 0.00
        trajReq.sz = 0.1
        xzSqRes = trjSRV(trajReq)
        print(xzSqRes.message)
    
    # complex square
    print("Boss Trajectory Tracking")
    checkMateFolder(folder_dir + "/boss/")
    trajReq.traj_type = "square"
    trajReq.exp_name = "/boss/results"
    trajReq.sx = 0.16
    trajReq.sy = 0.3
    trajReq.sz = 0.1
    bossRes = trjSRV(trajReq)
    print(bossRes.message)
    
        