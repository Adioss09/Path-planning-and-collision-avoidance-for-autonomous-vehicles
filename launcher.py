#!/usr/bin/env python3
import os
import sys
import subprocess

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def main():
    scenarios = [
        ("Cattle Crossing", "cattle"),
        ("Dense Market", "market"),
        ("4-Way Intersection", "intersection"),
        ("Unmarked Road", "unmarked")
    ]
    
    script_path = os.path.join(os.path.dirname(__file__), "scripts", "run_simulation.py")
    
    if not os.path.exists(script_path):
        print(f"Error: Could not find {script_path}")
        sys.exit(1)

    while True:
        clear_screen()
        print("="*50)
        print(" SIH26037 Autonomous Navigation Simulator ")
        print("="*50)
        print("\nPlease select a scenario to run:\n")
        
        for i, (name, _) in enumerate(scenarios, 1):
            print(f"  {i}. {name}")
            
        print("\n  D. Toggle Debug Mode")
        print("  Q. Quit")
        print("\n" + "="*50)
        
        choice = input("Enter your choice: ").strip().lower()
        
        debug_flag = "--debug" if "D" in choice.upper() else ""
        
        if choice == 'q':
            print("Exiting...")
            break
            
        if choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(scenarios):
                scenario_id = scenarios[idx][1]
                print(f"\nLaunching {scenarios[idx][0]} Scenario...\n")
                
                cmd = [sys.executable, script_path, "--scenario", scenario_id]
                
                # Ask about debug mode
                debug_choice = input("Enable debug visuals? (y/n): ").strip().lower()
                if debug_choice == 'y':
                    cmd.append("--debug")
                    
                try:
                    subprocess.run(cmd)
                except KeyboardInterrupt:
                    pass
                
                input("\nSimulation ended. Press Enter to return to the menu...")
            else:
                input("\nInvalid choice. Press Enter to try again...")
        else:
            input("\nInvalid input. Press Enter to try again...")

if __name__ == "__main__":
    main()
