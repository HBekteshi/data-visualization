# Data visualization project

This is a README file for the "Data visualization" project where implementation of a variety
of different graph layout algorithms is performed.

## Installation

To install the project, follow these steps:

1.Clone the repository to your local machine 

2.Install the required dependencies by running **'pipenv install --dev'**

3.Run the project using pipenv command: **'pipenv run python application.py'**. It is important to know that **'application.py'** should be run, for the application to function properly. 

## Usage 

To switch between different datasets included in the **data** folder of the repository, you can comment/uncomment the lines 20 to 28 in **main.py**.

Important thing to note is that when running datasets that contain subgraphs like *devonshiredebate_withonlytwoclusters.dot*, *polblogs_subgraphed.dot* in the *data* folder, the **subgraphs_included** variable in **main.py**(line 17) should be set to ***True***. 

## Contributing

If you would like to contribute to the project, please follow these steps:

- Fork the repository
- Create a new branch for your feature or bug fix
- Make your changes and commit them to your branch
- Push your changes to your forked repository
- Submit a pull request to the main repository
