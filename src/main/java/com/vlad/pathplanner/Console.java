package com.vlad.pathplanner;

public class Console {
    private static Console console = null;
    private Controller c = null;

    private Console()
    {

    }
    public void addLine(String s) {
        if (c == null) {
            System.err.println("Please set the controller first");
            System.exit(-1);
        }
        c.console.appendText(s);
    }

    public void clearConsole() {
        c.console.clear();;
    }

    public static synchronized Console getInstance()
    {
        if (console == null)
            console = new Console();
  
        return console;
    }
    public void setController(Controller c) {
        getInstance().c = c;
    }
}
