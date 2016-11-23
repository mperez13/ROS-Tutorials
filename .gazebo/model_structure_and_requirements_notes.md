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

####structure example:

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

###Database Config

- contains license info for models, a name for the database and a listt of all valid models
- NOTE: only required for online repositories

- format of this 'database.config':
```
<?xml version='1.0'?>
<database>
  <name>name_of_this_database</name>
  <license>Creative Commons Attribution 3.0 Unported</license>
  <models>
    <uri>file://model_directory</uri>
  </models>
</database>
```

###Model Config

- format of this `model.config`:

```
<?xml version="1.0"?>

<model>
  <name>My Model Name</name>
  <version>1.0</version>
  <sdf version='1.5'>model.sdf</sdf> //not required for URDFs; multiple <sdf> elements may be used in order to support  multiple SDF versions

  <author>
    <name>My name</name>
    <email>name@email.address</email>
  </author>

  <description>
    A description of the model
    Should include: 
      - what the model is (e.g. robot, table, cup)
      - What the plugins do (functionality of the model)
  </description>
  
  <depend>
    //This is optional
    All the dependencies for this model.
    This is typically other models.
  </depend>
  
  <model>
    //This is optional.
    <uri>URI of the model dependency</<uri> //required
    <version>version of the model</version> //required
  </model>
  
</model>
```

###Model SDF

- Each model requires 'model.sdf' file that contains the Simulator Description Format of the model.
- for more info: [SDF website](http://sdformat.org/)

###Model SDF.ERB

- Standard SDF file containing ruby code embedded
- used to programatically generate SDF files using [Embedded Ruby Code)(http://www.stuartellis.name/articles/erb/) templates
- NOTE: ruby conversion should be done manually ('erb model.sdf.erb > model.sdf') and the final 'model.sdf' file must be uploaded together w/ 'model.sdf.erb' (this one only for reference)

Example of 'sdf.erb' files: [gazebo_models repository](https://bitbucket.org/osrf/gazebo_models/src)

An easy ERB file is the [flocking.world.erb](https://bitbucket.org/osrf/gazebo/src/b54961341ffb938a9f99c9976aed50a771c95216/worlds/flocking.world.erb?at=default&fileviewer=file-view-default) which uses a simple loop.
 
