using System.Collections;
using UnityEngine;

namespace SpinMotion
{
    public class SkidTrail : MonoBehaviour
    {
        public float m_PersistTime;

        private IEnumerator Start()
        {
			while (true)
            {
                yield return null;

                if (transform.parent.parent == null)
                {
					Destroy(gameObject, m_PersistTime);
                }
            }
        }
    }
}
