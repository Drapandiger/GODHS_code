import sys

from scene_level.scene_level_search import scene_level_search

def main():
    
    print("""

___________                           __      _________                           .__        _________               __                  
\__    ___/____ _______  ____   _____/  |_   /   _____/ ____ _____ _______   ____ |  |__    /   _____/__.__. _______/  |_  ____   _____  
  |    |  \__  \\\\_  __ \/ ___\_/ __ \   __\  \_____  \_/ __ \\\\__  \\\\_  __ \_/ ___\|  |  \   \_____  <   |  |/  ___/\   __\/ __ \ /     \ 
  |    |   / __ \|  | \/ /_/  >  ___/|  |    /        \  ___/ / __ \|  | \/\  \___|   Y  \  /        \___  |\___ \  |  | \  ___/|  Y Y  \ 
  |____|  (____  /__|  \___  / \___  >__|   /_______  /\___  >____  /__|    \___  >___|  / /_______  / ____/____  > |__|  \___  >__|_|  /
               \/     /_____/      \/               \/     \/     \/            \/     \/          \/\/         \/            \/      \/ 

    """)


    while True:

        start_input = input("""\U0001F4E3\U0001F4E3\U0001F4E3\nWelcome to the target search system!\nInput 'help' to get the introduction.\nInput 'start' to start the system.\nInput 'exit' to drop out the system.\nNow you want: """)

        if start_input.lower() == "start":
            print("\u2B50 Starting the system...")
            try:
                scene_level_search()
            except Exception as e:
                print(f"\u274C An error occurred while starting the system: {e}")
            break
        elif start_input.lower() == "help":
            print("""\U0001F4A1\U0001F4A1\U0001F4A1\nThe target search system is a tool that can help you find the target you want to find.\nThe system is divided into four levels: scene level, room level, carrier level and item level.\nScene level: decide the scene.\nRoom level: divide the rooms in the scene.\nCarrier level: search for the carriers in the single room.\nItem level: search for the target in the single carrier. \n\U0001F4A1\U0001F4A1\U0001F4A1""")
        elif start_input.lower() == "exit":
            print("\U0001F6D1 You have dropped out the system.")
            break
        else:
            print("\u274C Invalid command. Please try again.")        
        
    print("\u2B50 Thank you for using the target search system, Goodbye!")
    sys.exit(0)

if __name__ == "__main__":
    main()


# \U0001F4E3 tongzhi
# \u2B50 print
# \U0001F4AC input
# \U0001F4A1 tixing
# \U0001F6D1 tingzhi
# \u274C cuowu