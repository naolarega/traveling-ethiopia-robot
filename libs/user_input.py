class UserInput:  
    @staticmethod
    def get_user_input():
        start_city = input("Enter the start city: ")
        goal_city = input("Enter the goal city: ")
        algorithm=input("Enter algorithm (bfs,dfs): ")
        return start_city, goal_city,algorithm
   
        default_start_city='AddisAbaba'
        default_goal_city='Moyale'
        choice=input(f"use default start {default_start_city} and goal {default_goal_city} state y/n : ")
        if choice=='n':
            start_city,goal_city= UserInput.get_user_input()
            return start_city,goal_city
        return default_start_city,default_goal_city