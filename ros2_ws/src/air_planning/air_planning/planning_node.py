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

#from math import sqrt

class planningNode(Node):

    def __init__(self):
      super().__init__('planning_node')

      self.get_logger().info("hi from planning Node")

      self.group = ReentrantCallbackGroup()

      relPath = Path.cwd()
      self.path = ""
      self.myDomain = open(relPath / "domain.pddl") #TODO CHANGE TO PDDL DIERCTOYRY AND FIX FILENAMES
      self.problemIndex = 0
      

        #print(self.data)
      self._action_server = ActionServer(
              self,
              Goals,
              '/goals_request',
              self.execute_nlp_callback)


    def execute_nlp_callback(self, goal_handle):
        self.get_logger().info('Planning Node: Executing nlp goal...')
        self.goals = goal_handle.goals
        
        self.data = self.queryDB()
        self.myProblem = self.makeProblem(self.data,self.goals)
        self.plan()
        
        result = Goals.Result()
        result.success = True
        return result


    

    def plan(self):
      self.cli = self.create_client(PlanRequest, 'plan_request')
      self.req = PlanRequest.Request()
     

      self.req.domain = myDomain.read() #MERGE OPEN AND READ
      self.req.problem = myProblem.read()
      self.req.format = "pddl"
      self.req.extensions = ['patterns']

      self.future = self.cli.call_async(self.req)
      rclpy.spin_until_future_complete(self, self.future)
      print("planner response done")
    #  print(self.future.result())
      return future.result()


    def queryDB(self):
      cli = self.create_client(QueryDatabase, 'kdb_server/sparql_query',callback_group=self.group)
      req = QueryDatabase.Request()

      req.graphname = "semanticobject"
      req.format = "json"
      req.query = "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis> \n PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties> \n SELECT ?obj_id ?class ?tags ?x ?y WHERE { ?obj_id a ?class ; \n properties:location [ gis:x ?x; gis:y ?y ]; properties:tags ?tags .}"   
      future = cli.call_async(req)
      rclpy.spin_until_future_complete(self,future)
      data = json.loads(future.result().result)
     
      return data

    #TODO MAP VENDING + COORDS to tell unique vendings apart
    def makeProblem(self, data, goals):
      objList = []
      initList = []
      goalList = []
      goBackToUser = False

      vendingIndex = 0
      #PARSE DATA FOR PROBLEM FILE
      for row in self.data["results"]["bindings"]:
        classes= row['class']['value']
        tags= row['tags']['value']
        x=row["x"]['value']
        y=row["y"]['value']
      #  print(classes, " - ", tags ," -- ", x ," --- ", y)
        objList.append("user - user")
        objList.append("r - robot")
        initList.append("(robotEmpty r)")
        initList.append("(robotAt r user)")

        if classes == "human":
          
          objList.append(tags + " - person")

        if classes == "office":
          objList.append("officeOf" + tags + " - office")

        if classes == "vendingmachine":
          objList.append("vendingMachine" + str(vendingIndex) + " - vending")
          initList.append("(vendingHas " + "vendingMachine" + str(vendingIndex) + " " + tags + ")" )
          vendingIndex = vendingIndex + 1
      #print(initList)

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
          
          initList.append("(personNeed " + person + goal.object + ")") #rename probably..
          goalList.append("(personHas " + person + " " + goal.object + ")")
          #content = goal.object
        f.write("(define (problem " + "problem" + self.problemIndex + ")\n")
        problemIndex = problemIndex + 1
        f.write("(:domain office)\n")
        f.write("(:objects\n")
  
        for line in objList:
          f.write(line)

        f.write("(:init\n")

        for line in initList:
          f.write(line)  

        f.write(")\n")
        f.write("(:goal (and\n")

        for line in goalList:
          f.write(line)

        f.write("\t))\n")
        f.write(")\n")

        fileName = "problem" + str(problemIndex-1) + ".pddl"
        problemFile = open(relPath / fileName ) #TODO CHANGE TO PDDL DIERCTOYRY 


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