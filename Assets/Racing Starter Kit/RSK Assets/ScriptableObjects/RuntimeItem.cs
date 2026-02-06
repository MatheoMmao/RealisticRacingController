using UnityEngine;

namespace SpinMotion
{
    public abstract class RuntimeItem<T> : ScriptableObject where T : MonoBehaviour
    {
        public T Item { get; private set; }

        public void Set(T thing)
        {
            if (Item != null)
            {
                Destroy(Item);
            }

            Item = thing;
        }

        public void Remove()
        {
            Item = default(T);
        }
    }
}