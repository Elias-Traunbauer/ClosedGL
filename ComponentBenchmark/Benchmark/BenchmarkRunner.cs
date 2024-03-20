using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ComponentBenchmark.Benchmark
{
    public class BenchmarkRunner
    {
        public static void RunBenchmarks()
        {
            var types = AppDomain.CurrentDomain.GetAssemblies()
                .SelectMany(s => s.GetTypes())
                .Where(p => p.IsClass);
            foreach (var type in types)
            {
                var methods = type.GetMethods().Where(m => m.GetCustomAttributes(typeof(BenchmarkAttribute), false).Length > 0);
                var beforeMethods = type.GetMethods().Where(m => m.GetCustomAttributes(typeof(BeforeEachBenchmarkAttribute), false).Length > 0);
                var beforeAllMethods = type.GetMethods().Where(m => m.GetCustomAttributes(typeof(BeforeAllBenchmarkAttribute), false).Length > 0);

                if (!methods.Any())
                {
                    continue;
                }
                var instance = Activator.CreateInstance(type);

                foreach (var item in beforeAllMethods)
                {
                    item.Invoke(instance, null);
                }

                void RunBeforeMethods()
                {
                    foreach (var method in beforeMethods ?? [])
                    {
                        method.Invoke(instance, null);
                    }
                }
                var toProcess = methods.GroupBy(m => m.DeclaringType);
                foreach (var classToProcess in toProcess)
                {
                    string className = classToProcess.Key!.Name;
                    Console.WriteLine("Running benchmarks for " + className);
                    foreach (var method in classToProcess)
                    {
                        List<double> times = new List<double>();
                        for (int i = 0; i < 10; i++)
                        {
                            RunBeforeMethods();
                            Stopwatch sw = new();
                            sw.Start();
                            method.Invoke(instance, null);
                            sw.Stop();
                            times.Add(sw.Elapsed.TotalMilliseconds);
                        }
                        Console.WriteLine("\t" + method.Name);
                        Console.WriteLine("\t\t" + times.Min() + "ms min");
                        Console.WriteLine("\t\t" + times.Max() + "ms max");
                        Console.WriteLine("\t\t" + times.Average() + "ms avg");
                        Console.WriteLine();
                    }
                    Console.WriteLine();
                }
            }
        }
    }
}
