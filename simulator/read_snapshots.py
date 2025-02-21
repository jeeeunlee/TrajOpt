import json
import os

def read_case_data(robot = "ra830a",
                       case_num = 1):
    data_path = f"test/testdata/8dof-collision/{robot}/case{case_num}.json"
    data_path = os.join(os.getcwd(), data_path)
    with open(data_path, 'r') as file:
        parsed_data = json.load(file)
    return parsed_data

def main():
    # Open the JSON file and read the data
    parsed_data = read_case_data()
    # Print out the parsed data
    print(json.dumps(parsed_data, indent=4))
   
if __name__ == "__main__":
    main()
    