import numpy as np
import time
import cv2 as cv
import heapq
import math
from queue import PriorityQueue

class Map:
    
    def __init__(self):
        # Visualizing the path by writing video
        self.height=500
        self.width=1200
        frameSize = (self.width, self.height)
        fourcc = cv.VideoWriter_fourcc('m','p','4','v')
        self.out = cv.VideoWriter('A_star_rohit_suresh_phase1.mp4', fourcc, 60, frameSize)
     
        # Initialising Variables 
        self.map=None
        self.obs_map=None
        self.clearance=5
        self.radius=0
        
    # Function to create the map and obstacle map
    def create_map(self):
        # Main map for display
        map=np.ones((self.height,self.width,3),dtype=np.uint8)
        map[:,:,0]=233
        map[:,:,1]=230
        map[:,:,2]=221

        # Obstacle map for path planning
        obs_map=np.ones((self.height,self.width),dtype=np.uint8)*255

        # Draw obstacles on the maps with clearance
        for i in range(self.width):
            for j in range(self.height):
                # Walls
                if i<self.clearance or i>1200-self.clearance or j<self.clearance or j>500-self.clearance:
                    map[j][i]=(2, 116, 189)
                    obs_map[j][i]=0
                # Rectangles
                if (i>=100-self.clearance and i<=175+self.clearance and j<=400) or (i>=275-self.clearance and i<=350+self.clearance and j>=100):
                    map[j][i]=(2, 116, 189)
                    obs_map[j][i]=0
                # Hexagon
                if ((150+self.clearance)/2)*abs(i-650)/(150+self.clearance)+100<=j<=300-((150+self.clearance)/2)*abs(i-650)/(150+self.clearance)+100 and 510-self.clearance<=i<=790+self.clearance:
                    map[j][i]=(2, 116, 189)
                    obs_map[j][i]=0
                # C- shaped Obstacle
                if (i>=900-self.clearance and i<=1100+self.clearance and j>=50-self.clearance and j<=125+self.clearance) or (i>=900-self.clearance and i<=1100+self.clearance and j>=375-self.clearance and j<=450+self.clearance) or (i>=1020-self.clearance and i<=1100+self.clearance and j>=50-self.clearance and j<=450+self.clearance):
                    map[j][i]=(2, 116, 189)
                    obs_map[j][i]=0
                
        # Draw obstacles on the maps          
        for i in range(self.width):
            for j in range(self.height):
                # Rectangles
                if (i>=100 and i<=175 and self.clearance<=j<=400-self.clearance) or (i>=275 and i<=350 and 500-self.clearance>=j>=100+self.clearance):
                    map[j][i]=(245, 114, 81)
                    obs_map[j][i]=0
                # Hexagon
                if (150/2)*abs(i-650)/150+105<=j<=300-(150/2)*abs(i-650)/150+95 and 510<=i<=790:
                    map[j][i]=(245, 114, 81)
                    obs_map[j][i]=0
                # C- shaped Obstacle
                if (i>=900 and i<=1100 and j>=50 and j<=125) or (i>=900 and i<=1100 and j>=375 and j<=450) or (i>=1020 and i<=1100 and j>=50 and j<=450):
                    map[j][i]=(245, 114, 81)
                    obs_map[j][i]=0

        
        return map,obs_map

    # Saving the map
    def generate_path_map(self):
        print("\nGenerating the map with path:")
        for i in range(len(path)-1):
            cv.arrowedLine(self.map,(path[i][0],path[i][1]),(path[i+1][0],path[i+1][1]),(0,0,255),1)
            
            # Start and end nodes
            cv.circle(self.map,(path[0][0],path[0][1]),2,(0,255,0),-1)
            cv.circle(self.map,(path[-1][0],path[-1][1]),2,(0,0,255),-1)
            self.out.write(self.map)
                
        for i in range(300):
            self.out.write(self.map)
        print("Map saved as A_Star_rohit_suresh_phase1.mp4\n")
        cv.waitKey(500)
        cv.destroyAllWindows()
        self.out.release()

class Node:
    
    def __init__(self):
        # Dictionary to store the nodes and their costs
        self.nodes={}                        
        
        # Thresholds for the nodes
        self.threshold_x = 0.5
        self.threshold_y = 0.5
        self.threshold_theta = 30
        
        # Visited nodes matrix
        self.visited_nodes=np.zeros((int(self.height//self.threshold_y)+1,int(self.width//self.threshold_x)+1,int(360//self.threshold_theta)+1),dtype=np.uint8)
        
        # Start and end nodes(Initialised to None)
        self.start_node=None            
        self.end_node=None     
        
        # Step size for the point robot
        self.step=10              

    # Function to insert a node into the nodes dictionary
    def insert_node(self,cost=None,node=None,parent=None):    
        if len(self.nodes)==0:
            self.nodes.update({(self.start_node):[None,0]})
            for i in range(600):
                for j in range(250):
                    if self.obs_map[j][i]==0 or self.obs_map[j][i]==128:
                        self.nodes.update({(i,j,0):[None,float('inf')]})
                        # self.visited_nodes[j][i][0]=1
        else:
            self.nodes.update({node:[parent,cost]})
            
    # Action functions to move the point robot in 8 directions
    def Actions(self,node):
        def action(angle):
            i,j,th=node
            theta = th + angle
            if theta > 180:
                theta -= 360
            theta=np.deg2rad((theta))
            i=np.ceil(i+self.step*np.sin(theta))
            j=np.ceil(j+self.step*np.cos(theta))
            th=(th+angle)%360
            i,j=int(i),int(j)
            cost=self.step
            if check_if_duplicate((i,j,th)):
               return None,None
            else:
                return (i,j,th),cost
        
        def check_if_duplicate(node):
            x,y,th=node
            if not self.is_valid(node):
                return True
            if self.visited_nodes[int(y/self.threshold_y)][int(x/self.threshold_x)][int(th/self.threshold_theta)]==1:
                return True
            return False
        
        return [action(-60),action(-30),action(0),action(30),action(60)]

    # Returns the parent node for a given node    
    def get_parent(self,node):
        return self.nodes[node][0]

class A_Star(Map,Node):
    
    def __init__(self):
        super().__init__()
        super(Map,self).__init__()
        
        # Getting user inputs
        self.get_user_inputs()
    
        # Inserting obstacle nodes into the nodes dictionary
        self.insert_node()
               
        # start_node and end_node
        cv.circle(self.map,(self.start_node[0],self.start_node[1]),2,(0,255,0),-1)
        cv.circle(self.map,(self.end_node[0],self.end_node[1]),2,(0,0,255),-1) 
        
    # A_Star function to generate the heap tree graph of the nodes using A_Star's Algorithm
    def A_Star_algorithm(self):
        open_list=[]
        closed_list=set()         
        tot_cost=self.total_cost(self.start_node)    
        heapq.heappush(open_list,(tot_cost,self.start_node))
        return_node=None
        c=0
        
        print("\nSearching for the path:")
        while open_list:
            c+=1
            _,current_node=heapq.heappop(open_list)
            current_c2c=self.nodes[current_node][1]  
            if current_node in closed_list:
                continue
            closed_list.add(current_node)    

            self.visited_nodes[int(current_node[1]/self.threshold_y)][int(current_node[0]/self.threshold_x)][int(current_node[2]/self.threshold_theta)]=1

            if current_node!=self.start_node:
                cv.arrowedLine(self.map,(self.nodes[current_node][0][0],self.nodes[current_node][0][1]),(current_node[0],current_node[1]),(0,0,0),1)
                if c%400==0:
                    self.out.write(self.map)
            
            if self.is_goal(current_node):
                open_list=None
                return_node=current_node
                break              
            
            for action in self.Actions(current_node):
                new_node,cost=action
                if new_node is not None and self.is_valid(new_node):
                    if self.visited_nodes[int(new_node[1]/self.threshold_y)][int(new_node[0]/self.threshold_x)][int(new_node[2]/self.threshold_theta)]==0:
                        self.visited_nodes[int(new_node[1]/self.threshold_y)][int(new_node[0]/self.threshold_x)][int(new_node[2]/self.threshold_theta)]=1
                        self.insert_node(cost+current_c2c,new_node,current_node)
                        tot_cost=self.total_cost(new_node)
                        heapq.heappush(open_list,(tot_cost,new_node))
                        
                        if current_c2c+cost<self.nodes[new_node][1]:
                            self.nodes[new_node][1]=current_c2c+cost
                            self.insert_node(current_c2c+cost,new_node,current_node)
                            heapq.heappush(open_list,(self.total_cost(new_node),new_node))

        time.sleep(0.01)
        return return_node

    def is_valid(self, node):
        # Check if the node is within map boundaries and not in an obstacle
        if node is None:
            return False
        within_bounds = node[0] in range(self.width) and node[1] in range(self.height)
        if not within_bounds:
            return False
        not_obstacle = self.obs_map[node[1]][node[0]] != 0
        return not_obstacle
    
    # Function to check if the current node is the goal node
    def is_goal(self,node):
        if math.sqrt((node[0] - self.end_node[0])**2 + (node[1] - self.end_node[1])**2) <= 1.5:
            if node[2]==self.end_node[2]:
                return True
        else:
            return False
    
    # Function to calculate the total cost of a node f(n)=h(n)+g(n): h(n)=euclidean heuristic from node to end node, g(n)=cost of node   
    def total_cost(self,node):
        i,j,_=node
        return math.sqrt((i-self.end_node[0])**2+(j-self.end_node[1])**2) +self.nodes[node][1]

    # Returns a path from the end_node to the start_node
    def construct_path(self):
        # Searching starts here
        path=[self.A_Star_algorithm()]
        if path[0] is not None:
            total_cost=self.nodes[path[0]][1]
            parent=self.get_parent(path[0])
            while parent is not None:
                path.append(parent)
                parent = self.get_parent(parent)
            path.reverse()
            print("Path found")
            return path,total_cost
        else:
            print("\nError: Path not found\nTry changing the orientation of the nodes")
            exit()

    # Getting user inputs
    def get_user_inputs(self):
        
        for i in range(2):
            str=["clearance","radius"]
            input_flag=True
            while input_flag:
                inp=input(f"Enter the {str[i]}: ")
                if not inp.isdigit():
                    print("Please enter a valid integer value.")
                    continue
                else:
                    input_flag=False
                    if i==0:
                        self.clearance=int(inp)
                    else:
                        self.radius=int(inp)
                        self.clearance+=self.radius
                                
        # Creating the map and obstacle map
        self.map,self.obs_map=self.create_map()
        
        input_flag=True
        while input_flag:
            step=input("Enter the step size(1-10): ")
            if not step.isdigit():
                print("Please enter a valid integer step size.")
                continue
            else:
                self.step=int(step)
            if self.step not in range(1,11):
                print("Please enter a valid step size.")
                continue
            else:
                self.step=int(step)
                input_flag=False
                
        input_flag=True
        print(f"Consider clearance of {self.clearance}.\nPlease enter the start and end nodes in the format 0 1 for (0,1)")
        
        for i in range(2):
            str=["start","end"]
            input_flag=True
            while input_flag:
                node=input(f"{str[i]} node:")
                try:
                    x,y,th=node.split()
                    x,y,th=int(x),self.height-int(y),int(th)
                except:
                    print("Please enter 3 valid integer coordinates.")
                    continue
                if x not in range(self.width) or y not in range(self.height) or th % 30 != 0:
                    print("Please enter valid coordinates.")
                    continue
                elif self.obs_map[y][x]==0:
                    print("Please enter a valid node(Node in obstacle place).")
                else:
                    input_flag=False
                    if i==0:
                        self.start_node=(x,y,th%360)
                    else:
                        self.end_node=(x,y,th%360)
    
# Main function
if __name__ == "__main__":
    start_time = time.time()

    # Creating the object of the class    
    a_Star=A_Star()

    # Generating the path and the total cost
    path,total_cost=a_Star.construct_path()   
    
    # Generating the path map
    a_Star.generate_path_map()
    
    path = [(path[i][0], a_Star.height - path[i][1],path[i][2]) for i in range(len(path))]
    # Printing the total cost and the path
    end_time = time.time()
    print(f"Time taken: {(end_time-start_time)/60} minutes.")
    print("\nPath cost: ",total_cost)
    print("\nPath: ",path)
