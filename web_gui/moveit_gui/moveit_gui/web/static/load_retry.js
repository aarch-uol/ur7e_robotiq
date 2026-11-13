export async function retryAsync(fn, { attempts = 4, baseMs = 250, maxMs = 4000,
                                       sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms)) } = {}) {
  let last;
  for (let i = 0; i < attempts; i++) {
    try { return await fn(i); }
    catch (e) {
      last = e;
      if (i + 1 >= attempts) break;
      await sleep(Math.min(maxMs, baseMs * 2 ** i));
    }
  }
  throw last;
}
