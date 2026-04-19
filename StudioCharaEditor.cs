using System;
using BepInEx;
using BepInEx.Configuration;
using BepInEx.Logging;
using HarmonyLib;
using System.Reflection;
using KKAPI;
using UnityEngine;

namespace StudioModsMSG
{
    [BepInPlugin(GUID, Name, Version)]
    [BepInDependency(KoikatuAPI.GUID, "1.4")]
    [BepInProcess("StudioNEOV2.exe")]
    public class StudioCharaEditor : BaseUnityPlugin
    {
        public const string GUID = "MSG.StudioCharaModsMSG.HS2";
        public const string Name = "Studio Chara Mods MSG";
        public const string Version = "0.0.1";

        public static StudioCharaEditor Instance { get; private set; }
        internal static new ManualLogSource Logger;

        // configs
        public static ConfigEntry<KeyboardShortcut> KeyShowUI { get; private set; }

        public static ConfigEntry<bool> VerboseMessage { get; private set; }

        public static ConfigEntry<int> UIXPosition { get; private set; }
        public static ConfigEntry<int> UIYPosition { get; private set; }
        public static ConfigEntry<int>  UIWidth      { get; private set; }
        public static ConfigEntry<int>  UIHeight     { get; private set; }
        public static ConfigEntry<bool> UIDarkMode   { get; private set; }
        public static ConfigEntry<float> UIOpacity   { get; private set; }

        /// <summary>Hold this key + Right-Mouse-Button to enter Manual Deformation mode on cloth.</summary>
        public static ConfigEntry<KeyboardShortcut> KeyManualDeform { get; private set; }

        //private ConfigEntry<string> configGreeting;
        //private ConfigEntry<bool> configDisplayGreeting;

        private void Awake()
        {
            Instance = this;
            Logger = base.Logger;

            // config
            KeyShowUI = Config.Bind("General", "Studio Chara Mods MSG UI shortcut key", new KeyboardShortcut(KeyCode.M, KeyCode.LeftShift), "Toggles the main UI on and off.");

            VerboseMessage = Config.Bind("Debug", "Print verbose info", false, "Print more debug info to console.");

            UIXPosition = Config.Bind("GUI", "Main GUI X position", 50, "X offset from left in pixel");
            UIYPosition = Config.Bind("GUI", "Main GUI Y position", 300, "Y offset from top in pixel");
            UIWidth     = Config.Bind("GUI", "Main GUI window width",  600, "Main window width, minimum 400, set it when UI is hidden.");
            UIHeight    = Config.Bind("GUI", "Main GUI window height", 400, "Main window height, minimum 150, set it when UI is hidden.");
            UIDarkMode  = Config.Bind("GUI", "Use dark theme", true, "Switch between night mode (black) and day mode (white).");
            UIOpacity   = Config.Bind("GUI", "Window opacity", 0.88f, "Background opacity of the UI window (0=invisible, 1=solid).");

            KeyManualDeform = Config.Bind("Cloth", "Manual Deform hold key",
                new KeyboardShortcut(KeyCode.D, KeyCode.LeftShift),
                "Hold this key and Right-Mouse-Button to enter Manual Deformation mode on cloth. " +
                "While held, drag the mouse near a cloth mesh to sculpt it with physics.");


            /*
            configGreeting = Config.Bind("General",   // The section under which the option is shown
                                        "GreetingText",  // The key of the configuration option in the configuration file
                                        "Hello, world!", // The default value
                                        "A greeting text to show when the game is launched"); // Description of the option to show in the config file
            configDisplayGreeting = Config.Bind("General.Toggles",
                                            "DisplayGreeting",
                                            true,
                                            "Whether or not to show the greeting text");
            */

            // init accessories plugin
            PluginMoreAccessories.Initialize();

            // start
            GameObject gameObject = new GameObject(Name);
            UnityEngine.Object.DontDestroyOnLoad(gameObject);
            CharaEditorMgr.Install(gameObject);

            // Patch
            //Harmony harmony = new Harmony(GUID);
            //harmony.PatchAll(Assembly.GetExecutingAssembly());
            
        }

    }
}
