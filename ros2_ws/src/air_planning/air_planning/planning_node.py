import rclpy
from rclpy.node import Node

#from std_msgs.msg import String

#from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry

from ros_hypertension_interfaces.srv import PlanRequest #import planner
from pathlib import Path
from ros2_kdb_msgs.srv import QueryDatabase
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import json
from rclpy.action import ActionServer
from air_interfaces.action import Goals
from air_interfaces.srv import GetEntities
from air_interfaces.msg import Entity

#from math import sqrt

class planningNode(Node):

    def __init__(self):
      super().__init__('planning_node')

      self.get_logger().info("hi from planning Node")
   
      self.group = ReentrantCallbackGroup()
      #self.coordMap
      
      self.relPath = Path.cwd()
      self.path = ""
      self.myDomain = open(self.relPath / "domain.pddl") #TODO CHANGE TO PDDL DIERCTOYRY AND FIX FILENAMES
      self.problemIndex = 0
      
      
      self.dbClient = self.create_client(GetEntities, 'get_entities')
      


        #print(self.data)
      self._action_server = ActionServer(
              self,
              Goals,
              '/goals_request',
              self.execute_nlp_callback)

    
    def queryDB(self):
        self.req = GetEntities.Request()
        self.future = self.dbClient.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().entities
   
    def runPlan(self, plan):
     
        actions = json.load(plan)
        tst = {
            "children":[],
            "common_params": {
            },
            "name": "seq",
            "params": {
            }
        }


        for action in actions:
            x = 0
            y = 0            
            #set x,y
            actionName = action[0] #setgoto for all except epxlore
            name = "" #name of office/person?
           
            #TODO coordinates get
            if actionName == "explore":
                pass

                node ={
                        "children": [],
                        "common_params": {},
                        "name": "explore",
                        "params": {
                            "kind": "klass", #TODO
                            "name": "tag", #TODO
                            "policy": "biggest",
                            "minsize": 15 
                        }
                    }

            else:
                to = action[2]
                
                if "officeof" in to:
                    to.replace("officeof","o")

                elif "vendingmachine" in to:
                    to.replace("vendingmachine","")
            
                x = self.coords[to][0]
                y = self.coords[to][1]

                node ={
                        "children": [],
                        "common_params": {},
                        "name": "drive-to",
                        "params": {
                            "p": {
                                "rostype": "Point",
                                "x": x,
                                "y": y,
                                "z": 0
                            }
                        }
                    }
                drive_to_human["children"].append(node)

        with open(self.relPath /'plan.json', 'w') as outfile:
            json.dump(plan, outfile)



            

    def execute_nlp_callback(self, goal_handle):
        self.get_logger().info('Planning Node: Executing nlp goal...')
        self.goals = goal_handle.request.goals

        self.get_logger().info("goals got")
        
        self.data = self.queryDB()
        
        self.get_logger().info("data base queried ")

        self.myProblem = self.makeProblem(self.data,self.goals)
        self.get_logger().info("problem got ")
        self.plan = self.getPlan()
        self.runPlan(self.plan)

        result = Goals.Result()
        result.success = True
        return result


    
    #runs planner
    def getPlan(self):
      self.cli = self.create_client(PlanRequest, 'plan_request')
      self.req = PlanRequest.Request()
     

      self.req.domain = self.myDomain.read() #MERGE OPEN AND READ
      self.req.problem = self.myProblem.read()
      self.req.format = "pddl"
      self.req.extensions = ['patterns']

      self.future = self.cli.call_async(self.req)
      rclpy.spin_until_future_complete(self, self.future)
      print("planner response done")
    #  print(self.future.result())
      return future.result()


 #   def queryDB(self):
 #     cli = self.create_client(QueryDatabase, 'kdb_server/sparql_query',callback_group=self.group)
 #     req = QueryDatabase.Request()

  #    req.graphname = "semanticobject"
  #    req.format = "json"
 #     req.query = "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis> \n PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties> \n SELECT ?obj_id ?class ?tags ?x ?y WHERE { ?obj_id a ?class ; \n properties:location [ gis:x ?x; gis:y ?y ]; properties:tags ?tags .}"   
  #    future = cli.call_async(req)
 #     rclpy.spin_until_future_complete(self,future)
  #    data = json.loads(future.result().result)

     # unique = { each['tags'] : each for each in data }.values()

     
      return data

    #TODO MAP VENDING + COORDS to tell unique vendings apart
    def makeProblem(self, data, goals):
        objList = []
        initList = []
        goalList = []
        goBackToUser = False

        objList.append("user - user")
        objList.append("r - robot")
        objList.append("coffee - content")
        objList.append("sandwich - content")
        initList.append("(robotEmpty r)")
        initList.append("(robotAt r user)")

        vendingIndex = 0
        #PARSE DATA FOR PROBLEM FILE
        self.vendingsAdded = []
        self.coords = {}
        for row in self.data:
            classes = row['klass']
            tags = row['tag']
            uuid = row['uuid']
            x=row["x"]
            y=row["y"]
            #print(classes, " - ", tags ," -- ", x ," --- ", y)
        
            
            if classes == "human":
            
                objList.append(tags + " - person")
                self.coords[tags] = [x,y]


            if classes == "office":
                objList.append("officeOf" + tags + " - office")
                self.coords["o" + tags] = [x,y]


            if classes == "vendingmachine":
                if uuid not in vendingsAdded:
                    objList.append("vendingMachine" + str(uuid) + " - vending")
                    vendingsAdded.append(uuid)
                    self.coords[uuid] = [x,y]

                for tag in tags:
                    initList.append("(vendingHas " + "vendingMachine" + str(uuid) + " " + tags + ")" )
               # vendingIndex = vendingIndex + 1
        #print(initList)
       
        fileName = "problem" + str(self.problemIndex-1) + ".pddl"

        for goal in goals:
            if goal.type == "goto":
            
                if goal.destination.type == "person":
                    goalList.append("(visited r " + goal.destination.name + ")")
                if goal.destination.type == "office":
                    goalList.append("(visited r officeOf" + goal.destination.name + ")")

                if goal.destination.type == "user":
                    goBackToUser = True

            
            if goal.type == "bring":
                person = ""
                if goal.destination.type == "user":
                    person = "user" #its gonna be fucked fix it
                if goal.destination.type == "person":
                    person = goal.destination.name
                if goal.destination.type == "office":
                    raise Exception("BRING TO OFFICE NOT YET IMPLEMENTED")
            
            initList.append("(personNeed " + person + " " + goal.object + ")") #rename probably..
            goalList.append("(personHas " + person + " " + goal.object + ")")
            #content = goal.object
            

        with open(fileName, 'w') as f:
            f.write("(define (problem " + "problem" + str(self.problemIndex) + ")\n")
            self.problemIndex += 1
            f.write("(:domain office)\n")
            f.write("(:objects\n")
        
            for line in objList:
                f.write(line + "\n")

            f.write(")\n")
            f.write("(:init\n")

            for line in initList:
                f.write(line + "\n")  

            f.write(")\n")
            f.write("(:goal (and\n")

            for line in goalList:
                f.write(line)

            f.write("\t))\n")
            f.write(")\n")

            
        problemFile = open(self.relPath / fileName ) #TODO CHANGE TO PDDL DIERCTOYRY 


        return problemFile #RETURN PROBLEM FILE
                    
        

      






def main(args=None):
    rclpy.init(args=args)

    planning_node = planningNode()

    rclpy.spin(planning_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.shutdown() 



if __name__ == '__main__':
    main()
