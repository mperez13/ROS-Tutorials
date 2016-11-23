#Model structure and requirements

Link to tutorial: http://gazebosim.org/tutorials?tut=model_structure&cat=build_robot

**Desciption:** Tutorial describes Gazebo's model directory structure and the necessary files within a model directory.

- A model:
  - may have one or more plugins, which affect the model's behavior
  - can represent anything from a simple shape to a complex robot; even the ground is a model

**The Model Database Repository** can be cloned using:
```
hg clone https://bitbucket.org/osrf/gazebo_models
```

##Model Database Structure

- **root** of a model database contains one directory for each model and a `database.config` file w/ info about the model database
- **model directory**: 
  - has a `model.config` file that contains meta data about the model
  - also contains the SDF for the model and any materials, meshes, and plugins
-sructure example:
`
- Database
  - *database.config*: Meta data about the database (is populated automatically from CMakeLists.txt)
  - *model_1*: A directory for model_1
    - *model.config*: Meta-data about model_1
    - *model.sdf*: SDF decription of the model
    - *model.sdf.erb*: Ruby embedded SDF model decription
    - *meshes*: directory for all COLLADA and STL files
    - *materials*: directory which should only contain the `textures` and `scripts` subdirectories
      - *textures*: directory for image files (jpg, png, etc)
      - *scripts*: directory for OGRE material scripts
    - *plugins*: directory for plugin source and header files
`

 
