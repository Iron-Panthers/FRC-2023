# FRC-2023

Iron Panthers repository for 2023 season code.

## Setting up

### Before working: make sure you have the 2023 tools:

- WPILib: download the latest release from [allwpilib](https://github.com/wpilibsuite/allwpilib/releases/latest). Run installer executable. Should include VS Code + extensions
  - using provided vscode is easier, but not ideal for everyone - it is optional
  - if you want to use your own vscode, just follow steps below to get extension running
- if you want to install VS Code extension separately, or for your own vscode instance
  - check if the version in the vscode store is up to date
    - if it is, install from there, make life easy
    - if not, do the following
  - download the latest .vsix release from [vscode-wpilib](https://github.com/wpilibsuite/vscode-wpilib/releases/latest)
  - open vscode
  - trigger command, which is ``ctrl shift p`` on linux
  - type install from and select vsix
  - select the vsix you downloaded
- Node.js: download 16 lts (or newer if you want / already have) from [node.js](https://nodejs.org/en/)
  - we use node for spotless linting on a few files with prettier
  - if node is not installed the build will fail
  - on windows, ***you will need to reboot for this to work***
  - on linux, install from your package manager

### Run a test build

Open command pallette (``ctrl shift p`` on linux) and type "build robot code" until you see the command. Running this command will build ***but not deploy*** your robot code, so you can check for syntax, linting, and library issues, among other things. If this says ``BUILD SUCCESSFUL`` it means all is working.

### Install additional recommended extensions

You should install a few vscode extensions to make our lives easier.

- Spotless Gradle
  - this will let you see linting errors in the editor
  - this will also automatically format your code to conform to linting rules
- Code Spell Checker (cSpell)
  - this will highlight likely typos
  - if you see something highlighted as a typo when it shouldn't be, like a robotics term, use the code action to ``Add: "xxx" to workspace settings`` so we all can benefit

## Getting work done

### Building to test code

If you want to build code and run tests, the build robot code command will do that. However, the vscode test task is also configured to do this, so running that with the keybinding you set will have the same effect.

To configure keybindings for testing and deploying, open the command pallette and run open keyboard shortcuts. Search ``tasks: run build`` to see build, and ``tasks: run test`` to see test. I configured my test task to be ``ctrl shift b ctrl shift j`` and my build task to be ``ctrl shift b ctrl shift b``. Vscode keybindings support chording, so you can put many things as the second layer of a binding.

### Deploying code

1. Turn on the robot
1. Wait for the radio light to be orange, letting you know wifi is on
1. Connect to robot wifi, probably named ``5026_(charles|devil|rat)`` with a name hopefully written on the radio (small white box)
1. Trigger the deploy action, either with command pallette or task keybinding
1. Connect the the network with a computer running driver station - it's only supported on windows, so if you want to deploy from your mac / linux device, use a driverstation in addition
1. Ensure the robot couldn't break things if it moved quickly / flung arm ect
1. Ask people near the robot if it is ok to enable if they are unaware of your plans
1. Say "Enabling!" loud enough everyone nearby can here you ***and wait for people to get away from the robot*** (very important)
1. Wait for someone to say "Clear" confirming that everyone is clear of the robot
1. Use the enable button on the driverstation to enable the robot
1. Ensure you or someone who understands the following is always close enough to the driverstation to stop the robot
1. Use the disable button to stop it when your done testing, or if the robot endangers itself / property / people because of your code
1. If something bad is happening, hit the enter to disable robot - you don't even need to have the driverstation focused. Don't bother looking for the disable button if someone is dying.
1. If something really bad is happening, hit the spacebar to disable robot and delete your code. This will require you to redeploy before the robot does anything again

### Writing code

When writing code, there are a few rules + changes to know.

- We have decided to stop using ``m_`` prefix for members of an object
- Constant blocks in ``Constants.java`` are PascalCase
- Constant variables in ``Constants.java`` are SCREAMING_SNAKE_CASE
- All subsystem filenames should be suffixed with Subsystem
  - ``ExampleSubsystem.java``
- All command filenames should be suffixed with Command
  - ``ExampleCommand.java``
- Controllers are named after the driver who uses them, ``jason`` and ``will``

### Code snippets

Code snippets are a vscode feature to allow easy typing of repetitive things. If snippets are added, they should be noted here - see ``.vscode/wpilib.code-snippets`` for examples if you want to write more. The prefix of a snippet is what you type to trigger it. To move between the places to fill in, use ``tab``.

| prefix | code |
| --- | --- |
| ``block`` | ``public static final class name {}`` |
| ``const`` | ``public static final type name = val;`` |
| ``thisprop`` | ``this.name = name;`` |
| ``fxper`` | ``motor.set(TalonFXControlMode.PercentOutput, percent);`` |
| ``newsub`` | ``private final SubsystemClassPascalCase subsystemClassCamelCase = new SubsystemClass();`` |
| ``vfxset`` | ``verify(motor, times(1).description(String.format("uh oh, %s", obj))).set(TalonFXControlMode.mode, value);`` |

### Collaborating on Code

To collaborate on code, keep a few things in mind:

- keep your feature branch fresh
  - this means merge dev into your branch when things are changed on dev
  - the worst case is that it takes a bit of time, but keeping your branch fresh will prevent large and difficult merge conflicts, because you will solve it as it happens
- unblock fast and seek collaboration
  - when you get stuck on something, "blocked", you should get unblocked quickly
  - while you could probably solve the issue yourself, you are surrounded by people with different experiences who may have already solved something similar
  - even if nobody knows the issue right away, summarizing the problem is usually a path to solution - looking at it in a new light
- critique code, not people
  - be kind to whoever wrote the code you take issue with
  - talk in terms of code, not who wrote it
    - "the code is crashing" vs "you crashed the robot"
  - similarly, don't be defensive about your code - you aren't your code
  - we should be mean to code, but not to people

If you pick up tips and tricks as you learn, make a pr and add them to the readme.
