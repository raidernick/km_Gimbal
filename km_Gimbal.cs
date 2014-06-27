/*
 * Author: dtobi
 * This work is shared under CC BY-NC-ND 3.0 license.
 * Non commercial, no derivatives, attribution if shared unmodified.
 * You may distribute this code and the compiled .dll as is.
 * 
 * Exception from the no-deriviates clause in case of KSP updates:
 * In case of an update of KSP that breaks the code, you may change
 * this code to make it work again and redistribute it under a different
 * class name until the author publishes an updated version. After a
 * release by the author, the right to redistribute the changed code
 * vanishes.
 * 
 * You must keep this boilerplate in the file and give credit to the author
 * in the download file as well as on the webiste that links to the file.
 * 
 * Should you wish to change things in the code, contact me via the KSP forum.
 * Patches are welcome.
 *
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

public class axisContribution
{
    public bool calculated = false;
    public double yawContributionX    = 0;
    public double pitchContributionX  = 0;
    public double rollContributionX   = 0;
    public double yawContributionY    = 0;
    public double pitchContributionY  = 0;
    public double rollContributionY   = 0;
    public double pitchYawSign = 0;
}

namespace km_Lib
{


	public class KM_Gimbal : PartModule
	{
		[KSPField(isPersistant = true, guiActive = false, guiActiveEditor = false, guiName = "Transform")]
		public string gimbalTransformName = "gimbal"; 

		[KSPField(isPersistant = true, guiActive = false, guiActiveEditor = false, guiName = "Yaw", guiUnits = "°")]//, UI_FloatRange(minValue = 0f, maxValue = 25.0f, stepIncrement = 1.0f)]
		public float yawGimbalRange = 1f; 

		[KSPField(isPersistant = true, guiActive = false, guiActiveEditor = false, guiName = "Pitch", guiUnits = "°")]//, UI_FloatRange(minValue = 0f, maxValue = 25.0f, stepIncrement = 1.0f)]
		public float pitchGimbalRange = 1f;

        [KSPField(isPersistant = true)]//, guiActive = true, guiActiveEditor = true, guiName = "Debug"), UI_Toggle(disabledText = "Disabled", enabledText = "Enabled")]
        public bool debug = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "X-Trim") , UI_FloatRange(minValue = -14f, maxValue = 14f, stepIncrement = 1f)]
        public float trimX      = 0;
        public float lastTrimX  = 0; // remember the last value to know when to update the editor

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Y-Trim") , UI_FloatRange(minValue = -14f, maxValue = 14f, stepIncrement = 1f)]
        public float trimY      = 0;
        public float lastTrimY  = 0; // remember the last value to know when to update the editor

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Trim"),
            UI_Toggle(disabledText="Disabled", enabledText="Enabled")]
        public bool  enableTrim = true;

        //[KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "DebugLevel"),UI_FloatRange(minValue = 0f, maxValue = 5f, stepIncrement = 1f)]
        public float debugLevel = 0;

		[KSPField(isPersistant = false, guiActive = false, guiActiveEditor = false, guiName = "CurrentYaw")]
		private float currentYaw = 0;

		[KSPField(isPersistant = false, guiActive = false, guiActiveEditor = false, guiName = "CurrentPitch")]
		private float currentPitch = 0;

		[KSPField(isPersistant = false, guiActive = false, guiActiveEditor = false, guiName = "CurrentRoll")]
		private float currentRoll = 0;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Constrain gimbal") , UI_FloatRange(minValue = 0f, maxValue = 14f, stepIncrement = 1f)]
        public float gimbalConstrain      = 14;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = false, guiName = "Gimbal"),
			UI_Toggle(disabledText="Disabled", enabledText="Enabled")]
		public bool enableGimbal = true;

        // [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = false, guiName = "Performance"),
        //    UI_Toggle(disabledText="Off", enabledText="Tweaked")]
        public bool performanceEnabled = true;

        //[KSPField(isPersistant = true, guiActive = false, guiActiveEditor = false, guiName = "Perf Factor"), UI_FloatRange(minValue = 1, maxValue = 30, stepIncrement = 1f)]
        public float perfFactor = 10;

		[KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Method"),
			UI_Toggle(disabledText="Precise", enabledText="Smooth")]
		public bool enableSmoothGimbal  = false;

		[KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Speed"),
			UI_FloatRange(minValue =1f, maxValue = 100.0f, stepIncrement = 1f)]
		public float responseSpeed = 60;
		public float maxResponseSpeed = 100f;

        [KSPField(isPersistant = false, guiActive = false, guiActiveEditor = false)]
        public bool invertPitch = false;

        [KSPField(isPersistant = false, guiActive = false, guiActiveEditor = false)]
        public bool invertYaw = false;

        [KSPField(isPersistant = false, guiActive = false, guiActiveEditor = false)]
        public bool invertRoll = false;

        bool isRunning = false; 

        // save transforms and initial rotations
        public List<UnityEngine.Transform>  gimbalTransforms    = new List<UnityEngine.Transform>();
        public List<UnityEngine.Quaternion> initRots            = new List<UnityEngine.Quaternion>();

        // Cache for axis contribution
        public List<axisContribution>       aContrib            = new List<axisContribution>();

        // calculation limiter for reducing computations
        private int calcCounter = 0;



        private ModuleEngines engine = null;
        private ModuleEnginesFX engineFX = null;
        private bool isMultiMode = false;

		private void printd(int debugPriority, string text){
			if (debug && debugPriority <= debugLevel)
				print ("d"+debugPriority+" "+text);
		}

		[KSPEvent(guiName = "Toggle Debug", guiActive = false)]
		public void dbg_run (){
            debugLevel = 0;
		}

		[KSPAction("Toggle Gimbal")]
		public void toggleGimbal (KSPActionParam param){
			enableGimbal = !enableGimbal;
            resetTransform ();  
		}

        [KSPAction("X Trim +")]
        public void plusTrimX (KSPActionParam param){
            trimX=trimX+1;
        }
        [KSPAction("X TrimX -")]
        public void minusTrim (KSPActionParam param){
            trimX=trimX-1;
        }

        [KSPAction("X TrimX +5")]
        public void plus5Trim (KSPActionParam param){
            trimX=trimX+5;
        }
        [KSPAction("X TrimX -5")]
        public void minus5Trim (KSPActionParam param){
            trimX=trimX-5;
        }

        [KSPAction("Y Trim +")]
        public void plusTrimY (KSPActionParam param){
            trimY=trimY+1;
        }
        [KSPAction("Y Trim -")]
        public void minusTrimY (KSPActionParam param){
            trimY=trimY-1;
        }

        [KSPAction("Y Trim +5")]
        public void plus5TrimY (KSPActionParam param){
            trimY=trimY+5;
        }
        [KSPAction("Y Trim -5")]
        public void minus5TrimY (KSPActionParam param){
            trimY=trimY-5;
        }
            
        [KSPAction("Toggle Trim")]
        public void toggletTrim (KSPActionParam param){
            enableTrim = !enableTrim;
		}

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Roll")]
        public bool enableRoll = false;
        [KSPField(isPersistant = true)]
        private bool autoSetRoll = true;

        [KSPEvent(guiName = "Toggle Roll", guiActive = true, guiActiveEditor = true)]
        public void toggleRoll(){
            autoSetRoll = false;
            setRoll (!enableRoll);
        }

        public void setRoll(bool enabled){
            enableRoll = enabled;
            if (Events ["toggleRoll"] != null) {
                if (enabled) {
                    Events ["toggleRoll"].guiName = "Deactivate roll";
                } else {
                    Events ["toggleRoll"].guiName = "Activate roll";
                }
            }
       }



        private void resetTransform() {
            for (int i = 0; i < gimbalTransforms.Count; i++) {
                gimbalTransforms [i].localRotation = initRots [i];
            }
        }

		public override string GetInfo ()
		{
			return "Yaw Gimbal\n"+yawGimbalRange+"\n" +
				"Pitch Gimbal\n"+pitchGimbalRange+"\n"+
                "KM Gimbal plugin by dtobi";
		}



		public override void OnStart (StartState state)
		{
			

            foreach (UnityEngine.Transform transform in part.FindModelTransforms(gimbalTransformName)) {
                gimbalTransforms.Add(transform);
                printd (0, "Adding transform:" + transform);
                printd (0, "Rots:" + transform.localRotation);
                initRots.Add (transform.localRotation);
                aContrib.Add (new axisContribution ());

            }

            if (state == StartState.Editor) {
                print ("Roll is enabled?: " + enableRoll);
                this.part.OnEditorAttach += OnEditorAttach;
                this.part.OnEditorDetach += OnEditorDetach;
                this.part.OnEditorDestroy += OnEditorDestroy;
                setRoll (enableRoll);
                OnEditorAttach ();



            } else {

                // initialize the random to not calc at once
                calcCounter = (int)UnityEngine.Random.Range (0, 100);

                //first try to find a engineFX... we like pretty flames
                var engineFXs = this.part.GetComponentsInChildren <ModuleEnginesFX> ();
                if (engineFXs.Count() > 0) {
                    engineFX = engineFXs [0];
                    // if this is a multimode engine, we deactivate the automatic gimbal shutoff
                    isMultiMode = (engineFXs.Count() > 1);
                }

                if (engineFX == null) {
                    print ("Gimbal ERROR: ModuleEngineFX not found!");
                    var engines = this.part.GetComponentsInChildren <ModuleEngines> ();
                    if (engines.Count () > 0) {
                        engine = engines [0];
                        isMultiMode = (engines.Count() > 1);
                    } else {
                        print ("Gimbal ERROR: No engine module found. ModuleEngines not found!");
                
                    }

                }
                if (isMultiMode)
                    print ("This is a multimode engine. Deactivated automatic gimbal shutoff");

            }
			base.OnStart (state);
		}

		/// <summary>
		/// Determine the signed angle between two vectors, with normal 'n'
		/// as the rotation axis.
		/// Code by Tinus: http://forum.unity3d.com/threads/51092-need-Vector3-Angle-to-return-a-negtive-or-relative-value
		/// </summary>
		public static double AngleSigned(Vector3 v1, Vector3 v2, Vector3 n)
		{
			return Mathf.Atan2(
				Vector3.Dot(n, Vector3.Cross(v1, v2)),
				Vector3.Dot(v1, v2));
		}





		private void OnEditorAttach()
		{
			RenderingManager.AddToPostDrawQueue(99, updateEditor);
            if (autoSetRoll) {
                setRoll(this.part.symmetryCounterparts.Count () != 0);
                print ("KM_Gimbal: Setting roll for auto set:");
            } else {
                setRoll(enableRoll);
                print ("KM_Gimbal: Leaving roll as is:"+enableRoll);
            }
            //print("KM_Gimbal: attaching");
		}

		private void OnEditorDetach()
		{

			RenderingManager.RemoveFromPostDrawQueue(99, updateEditor);
            //print("KM_Gimbal: OnEditorDetach");
        }

		private void OnEditorDestroy()
		{
			RenderingManager.RemoveFromPostDrawQueue(99, updateEditor);
            //print("KM_Gimbal: OnEditorDestroy");

		}

        public override void OnUpdate ()
        {
            // enable activation on action group withou staging
            if (!isMultiMode && ((engine != null && engine.getIgnitionState)
                ||  (engineFX != null && engineFX.getIgnitionState)) && !isRunning) {
                if(engine != null ) print ("Forcing activation " + engine.getIgnitionState);
                if(engineFX != null ) print ("Forcing activation " + engineFX.getIgnitionState);
                this.part.force_activate ();
            }
        }

		public override void OnFixedUpdate(){
			if (HighLogic.LoadedSceneIsEditor)
				return;

            updateFlight ();
		}

		private void updateEditor(){
            if (trimX != lastTrimX || trimY != lastTrimY) {
                Vector3 rotVec = new Vector3 (
                    Mathf.Clamp(trimX, pitchGimbalRange*-1, pitchGimbalRange),
                    Mathf.Clamp(trimY, yawGimbalRange*-1, yawGimbalRange), 0f);
				for (int i = 0; i < gimbalTransforms.Count; i++) {
					gimbalTransforms [i].localRotation = initRots [i];
					gimbalTransforms [i].Rotate (rotVec);
				}
			}
			lastTrimX = trimX;
            lastTrimY = trimY;
		}

		private void updateFlight(){
            if (!enableGimbal || (!isMultiMode && ((engine != null && !engine.EngineIgnited) || 
                (engineFX != null && !engineFX.EngineIgnited)))) {
                if (isRunning) {
                    resetTransform ();
                    isRunning = false;
                        print("Engine not running. Turining off gimbal");
                }
                return;           
            }
			    
            isRunning = true;



            calcCounter++;

            float maxPitchGimbalRange = Math.Min (pitchGimbalRange, gimbalConstrain);
            float maxYawGimbalRange = Math.Min (pitchGimbalRange, gimbalConstrain);

            currentPitch = vessel.ctrlState.pitch * (invertPitch?-1:1);
            currentYaw   = vessel.ctrlState.yaw   * (invertYaw?-1:1);
            currentRoll  = vessel.ctrlState.roll  * (invertRoll?-1:1);

			for (int i = 0; i < gimbalTransforms.Count; i++) {
				// save the current gimbal rotation and restore defaults
				UnityEngine.Quaternion savedRot = gimbalTransforms [i].localRotation;
				gimbalTransforms [i].localRotation = initRots [i];


				if (debug) {
					printd (2, "Engine right:" + gimbalTransforms [i].right);
                    printd (2, "Vessel right:" + vessel.ReferenceTransform.right);

					printd (2, "Engine forward:" + gimbalTransforms [i].forward);
                    printd (2, "Vessel forward:" + vessel.ReferenceTransform.forward);

					printd (2, "Engine up:" + gimbalTransforms [i].up);
                    printd (2, "Vessel up:" + vessel.ReferenceTransform.up);
				}

   

                if (!performanceEnabled || !aContrib [i].calculated || calcCounter % (int)perfFactor == 0) {
                   
                  

                    // find the vector between engine and vessel
                    Vector3 vesselCenterOfMass = vessel.findWorldCenterOfMass ();
                    Vector3 center = vesselCenterOfMass - gimbalTransforms [i].position;

                    // Determine the contribution of the engine's x and y rotation on the yaw, pitch, and roll
                    double yawAngleX = AngleSigned (gimbalTransforms [i].right, vessel.ReferenceTransform.right, gimbalTransforms [i].forward) * -1;
                    double pitchAngleX = AngleSigned (gimbalTransforms [i].right, vessel.ReferenceTransform.forward, gimbalTransforms [i].forward);
                    double rollAngleX = (enableRoll ? AngleSigned (gimbalTransforms [i].up, center, gimbalTransforms [i].forward) : 0);

                    double yawAngleY = AngleSigned (gimbalTransforms [i].up, vessel.ReferenceTransform.right, gimbalTransforms [i].forward);
                    double pitchAngleY = AngleSigned (gimbalTransforms [i].up, vessel.ReferenceTransform.forward, gimbalTransforms [i].forward) * -1;
                    double rollAngleY = (enableRoll ? AngleSigned (gimbalTransforms [i].right, center, gimbalTransforms [i].forward) : 0);

                    aContrib [i].yawContributionX = Math.Sin (yawAngleX);
                    aContrib [i].pitchContributionX = Math.Sin (pitchAngleX);
                    aContrib [i].rollContributionX = (enableRoll ? Math.Sin (rollAngleX) : 0);

                    aContrib [i].yawContributionY = Math.Sin (yawAngleY);
                    aContrib [i].pitchContributionY = Math.Sin (pitchAngleY);
                    aContrib [i].rollContributionY = (enableRoll ? Math.Sin (rollAngleY) : 0);


                    // determine if we are in front or behind the CoM tgo flip axis (Goddard style rockets)
                    //var pitchYawSign = Math.Sign (Vector3.Dot (vessel.findWorldCenterOfMass () - part.rigidbody.worldCenterOfMass, vessel.transform.up));
                    aContrib [i].pitchYawSign = Math.Sign (Vector3.Dot (center, vessel.ReferenceTransform.up));
                    aContrib [i].calculated = true;
                }

                var rotX = Mathf.Clamp ((enableTrim?trimX:0) + (float)(currentYaw * aContrib[i].yawContributionX * aContrib[i].pitchYawSign + currentPitch * aContrib[i].pitchContributionX * aContrib[i].pitchYawSign + currentRoll * aContrib[i].rollContributionX) * maxPitchGimbalRange, -maxPitchGimbalRange, maxPitchGimbalRange);
                var rotY = Mathf.Clamp ((enableTrim?trimY:0) +(float)(currentYaw * aContrib[i].yawContributionY * aContrib[i].pitchYawSign + currentPitch * aContrib[i].pitchContributionY * aContrib[i].pitchYawSign + currentRoll * aContrib[i].rollContributionY) * maxYawGimbalRange * -1, -maxYawGimbalRange, maxYawGimbalRange);

				Vector3 rotVec = new Vector3 ((float)rotX, (float)rotY, 0f);

				if (enableSmoothGimbal) {
					printd (3, "Animated Gimbal");
					gimbalTransforms [i].Rotate (rotVec);
					gimbalTransforms [i].localRotation = Quaternion.RotateTowards (savedRot, gimbalTransforms [i].localRotation, responseSpeed * TimeWarp.fixedDeltaTime);
				} else {
					printd (3, "Lerp Gimbal");
					gimbalTransforms [i].Rotate (rotVec);
					gimbalTransforms [i].localRotation = Quaternion.Lerp (savedRot, gimbalTransforms [i].localRotation, responseSpeed / maxResponseSpeed);
				}

				if (debug) {
					printd (1, "rotX:" + rotX);
					printd (1, "rotY:" + rotY);


                   
                    printd (2, "ycX:" + aContrib[i].yawContributionX + " pcX:" + aContrib[i].pitchContributionX + " rcX:" + aContrib[i].rollContributionX);
                    printd (2, "ycY:" + aContrib[i].yawContributionY + " pcY:" + aContrib[i].pitchContributionY + " rcX:" + aContrib[i].rollContributionY);
					printd (2, "vessel.transform.right:" + vessel.transform.right);
					printd (2, "vessel.transform.forward:" + vessel.transform.forward);
					printd (2, "vessel.transform.up:" + vessel.transform.up);
					printd (1, "vessel.ctrlState.pitch" + vessel.ctrlState.pitch);
					printd (1, "vessel.ctrlState.yaw" + vessel.ctrlState.yaw);
					printd (1, "vessel.ctrlState.roll" + vessel.ctrlState.roll);
				}
			}
		}
	}
}


