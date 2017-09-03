# ACIOLauncher

A launcher that takes input from an ACIO reader attached via serial to a cabinet.

This will initialize either slotted or wavepass reader hardware, plugged into any standard COM port on actual BEMANI hardware. Once initialized, it will present a menu of games to launch, based on a configured ini file. If a game is not selected, it will boot the first option after 60 seconds. Note that it currently only handles the first 9 games listed in the ini on account of there being only 9 non-zero buttons on a standard reader. Its possible that in a future patch I will implement paging.

The project provided is a Visual Studio 2008 project that produces a statically-compiled executable which is ready to run on embedded XP. This is suitable for execution on the operating system that comes with various BEMANI games.

The ini file format is simple. For each game, there should be a section which is the game name. For each section, there should be a "launch" key which points to the full path of the executable or batch file to execute when selecting this option. An example is below:

```
[Lincle]
launch=D:\LincleData\contents\gamestart.bat

[Resort Anthem]
launch=D:\RAData\contents\gamestart.bat

[Sirius]
launch=D:\Sirius\contents\gamestart.bat
```

To corretly execute the built code, run the executable with one parameter specifying the location of the INI file. Optionally, you can add the `--debug` attribute for additional debugging. An example invocation is as follows.

```
ACIOLauncher.exe games.ini
```