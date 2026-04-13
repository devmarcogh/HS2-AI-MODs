using AIChara;
using CharaCustom;
using KKAPI.Studio;
using Studio;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace StudioModsMSG
{
    class CharaEditorMgr : MonoBehaviour
    {
        public BaseUI gui;
        public Dictionary<OCIChar, CharaEditorController> charaEditorCtrlDict = new Dictionary<OCIChar, CharaEditorController>();
        public ModuleRegistry Modules { get; } = new ModuleRegistry();
        public SelectionContext ActiveSelection { get; private set; } = SelectionContext.Empty;

        public static CharaEditorMgr Instance { get; private set; }

        public static CharaEditorMgr Install(GameObject container)
        {
            if (CharaEditorMgr.Instance == null)
            {
                CharaEditorMgr.Instance = container.AddComponent<CharaEditorMgr>();
            }
            return CharaEditorMgr.Instance;
        }

        public bool VisibleGUI
        {
            get => gui.VisibleGUI;
            set => gui.VisibleGUI = value;
        }

        private void Awake()
        {
            
            RegisterModule(new SoftBodyModule());
            RegisterModule(new ClothPhysicsEditorModule());
        }

        private void Start()
        {
            StartCoroutine(LoadingCo());
        }

        //[Warning: Unity Log] OnLevelWasLoaded was found on ConsolePlugin
        //This message has been deprecated and will be removed in a later version of Unity.
        //Add a delegate to SceneManager.sceneLoaded instead to get notifications after scene loading has completed
        private IEnumerator LoadingCo()
        {
            yield return new WaitUntil(() => StudioAPI.StudioLoaded);
            // Wait until fully loaded
            yield return null;

            // start ui
            gui = new GameObject("GUI").AddComponent<BaseUI>();
            gui.transform.parent = base.transform;
            gui.VisibleGUI = false;
            //Console.WriteLine("StudioCharaEditor CharaEditorMgr Started.");

            // check extra plugins
        }

        public void ResetGUI()
        {
            gui.ResetGui();
        }

        public void HouseKeeping(bool isVisible)
        {
            // release deleted controller
            if (isVisible)
            {
                List<OCIChar> invalidTargets = new List<OCIChar>();
                foreach (OCIChar ociChar in charaEditorCtrlDict.Keys)
                {
                    if (ociChar.charInfo == null)
                    {
                        invalidTargets.Add(ociChar);
                    }
                }

                foreach (OCIChar invalidTarget in invalidTargets)
                {
                    Console.WriteLine("Remove controller for deleted chara");
                    charaEditorCtrlDict.Remove(invalidTarget);
                }
            }

            // housekeeping for controller
            if (isVisible)
            {
                foreach (var ctrl in charaEditorCtrlDict.Values)
                {
                    ctrl.RefreshAccessoriesListIfExpired();
                }
            }
        }

        public CharaEditorController GetEditorController(OCIChar ociTarget)
        {
            if (ociTarget == null)
            {
                return null;
            }
            if (!charaEditorCtrlDict.ContainsKey(ociTarget))
            {
                charaEditorCtrlDict[ociTarget] = new CharaEditorController(ociTarget);
                charaEditorCtrlDict[ociTarget].Initialize();
            }
            return charaEditorCtrlDict[ociTarget];
        }

        public CharaEditorController GetEditorController(ObjectCtrlInfo ociTarget)
        {
            return GetEditorController(ociTarget as OCIChar);
        }

        public CharaEditorController GetEditorController(ChaControl chaCtrl)
        {
            foreach (OCIChar ociChar in charaEditorCtrlDict.Keys)
            {
                if (ociChar.charInfo == chaCtrl)
                {
                    return charaEditorCtrlDict[ociChar];
                }
            }
            return null;
        }

        public bool RegisterModule(IEditorModule module)
        {
            return Modules.Register(module);
        }

        public SelectionContext ResolveSelection(TreeNodeObject node)
        {
            if (node == null)
            {
                return SelectionContext.Empty;
            }

            ObjectCtrlInfo target = null;
            var dic = Studio.Studio.Instance.dicInfo;
            if (dic.ContainsKey(node))
            {
                target = dic[node];
            }

            if (target == null)
            {
                return SelectionContext.Empty;
            }

            return new SelectionContext(node, target, target as OCIChar, Modules.GetCompatibleModules(target));
        }

        public void UpdateSelection(TreeNodeObject node)
        {
            ActiveSelection = ResolveSelection(node);
        }

        static public bool SetCustomBase(ChaControl chaCtrl)
        {
            // check and init CustomBase
            if (Singleton<CustomBase>.Instance == null)
            {
                try
                {
                    CustomBase dummyCustomBase = CharaEditorMgr.Instance.gameObject.AddComponent<CustomBase>();
                }
                catch (Exception ex)
                {
                    Console.WriteLine("This is an expected exception for creating a CustomBase in studio: " + ex.Message);
                }

                // re-check
                if (Singleton<CustomBase>.Instance == null)
                {
                    StudioCharaEditor.Logger.LogError("Fail to create CustomBase.");
                    return false;
                }
            }

            try
            {
                Singleton<CustomBase>.Instance.chaCtrl = chaCtrl;
                return true;
            }
            catch (Exception ex)
            {
                StudioCharaEditor.Logger.LogError("Fail to set CustomBase.chaCtrl: " + ex.Message);
                return false;
            }
        }
    }
}
