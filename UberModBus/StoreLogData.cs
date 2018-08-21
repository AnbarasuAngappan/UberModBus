using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace UberModBus
{
    class StoreLogData
    {
        private static object syncObject = new object();
        private string filename = (string)null;
        private static volatile StoreLogData instance;

        private StoreLogData()
        {
        }

        public static StoreLogData Instance
        {
            get
            {
                if (StoreLogData.instance == null)
                {
                    lock (StoreLogData.syncObject)
                    {
                        if (StoreLogData.instance == null)
                            StoreLogData.instance = new StoreLogData();
                    }
                }
                return StoreLogData.instance;
            }
        }

        public void Store(string message)
        {
            if (this.filename == null)
                return;
            using (StreamWriter streamWriter = new StreamWriter(this.Filename, true))
                streamWriter.WriteLine(message);
        }

        public void Store(string message, DateTime timestamp)
        {
            try
            {
                using (StreamWriter streamWriter = new StreamWriter(this.Filename, true))
                    streamWriter.WriteLine(timestamp.ToString("dd.MM.yyyy H:mm:ss.ff ") + message);
            }
            catch (Exception ex)
            {
            }
        }

        public string Filename
        {
            get
            {
                return this.filename;
            }
            set
            {
                this.filename = value;
            }
        }
    }
}

