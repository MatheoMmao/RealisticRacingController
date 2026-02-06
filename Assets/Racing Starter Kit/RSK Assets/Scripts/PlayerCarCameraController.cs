using UnityEngine;
/// <summary>
/// it provides a basic smooth follow camera to the player car
/// replace with your own car camera controller
/// </summary>
namespace SpinMotion
{
    public class PlayerCarCameraController : MonoBehaviour
    {
        public GameObject playerCar;
        public float posX, posY, posZ;
        public float rotX;

        [Tooltip("If car speed is below this value, then the camera will default to looking forwards.")]
        public float rotationThreshold = 1f;

        [Tooltip("How closely the camera follows the car's position. The lower the value, the more the camera will lag behind.")]
        public float cameraStickiness = 10.0f;

        [Tooltip("How closely the camera matches the car's velocity vector. The lower the value, the smoother the camera rotations, but too much results in not being able to see where you're going.")]
        public float cameraRotationSpeed = 5.0f;

        private Camera carCamera;
        private Transform rootNode;
        private Transform car;
        private Rigidbody carPhysics;

        private void Start()
        {
            if (playerCar == null)
            {
                Debug.LogError("No player car assigned to cameras");
                return;
            }

            carCamera = GetComponent<Camera>();
            rootNode = GetComponent<Transform>();
            car = playerCar.GetComponent<Transform>();
            carPhysics = playerCar.GetComponent<Rigidbody>();
        }
        
        private void LateUpdate()
        {
            carCamera.transform.Translate(posX, posY, posZ);
            float rotZ = GetComponent<Camera>().transform.eulerAngles.z;
            carCamera.transform.Rotate(rotX, 0, -rotZ);
            
            // Moves the camera to match the car's position.
            rootNode.position = Vector3.Lerp(rootNode.position, car.position, cameraStickiness * Time.fixedDeltaTime);

            // If the car isn't moving, default to looking forwards. Prevents camera from freaking out with a zero velocity getting put into a Quaternion.LookRotation
            Quaternion look;

            if (carPhysics.linearVelocity.magnitude < rotationThreshold)
                look = Quaternion.LookRotation(car.forward);
            else
                look = Quaternion.LookRotation(carPhysics.linearVelocity.normalized);

            // Rotate the camera towards the velocity vector.
            look = Quaternion.Slerp(rootNode.rotation, look, cameraRotationSpeed * Time.fixedDeltaTime);
            rootNode.rotation = look;
        }
    }
}