#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */

static struct lock *intersectionLock;
static struct cv *nw;
static struct cv *ns;
static struct cv *ne;
static struct cv *sn;
static struct cv *sw;
static struct cv *se;
static struct cv *en;
static struct cv *ew;
static struct cv *es;
static struct cv *wn;
static struct cv *ws;
static struct cv *we;

volatile int nw_lk;
volatile int ns_lk;
volatile int ne_lk;
volatile int sn_lk;
volatile int sw_lk;
volatile int se_lk;
volatile int en_lk;
volatile int ew_lk;
volatile int es_lk;
volatile int wn_lk;
volatile int ws_lk;
volatile int we_lk;

static struct cv *cur_cv;
volatile int cur_lk;


// locked works when a car try to pass the intersection(with origin and destination), then lock all
// the other cv which could be crash with this car.

static void locked(Direction origin, Direction destination) {
  if (origin == north && destination == south) {
    we_lk++;
    ew_lk++;
    es_lk++;
    ws_lk++;
    wn_lk++;
    sw_lk++;

  } else if (origin == north && destination == west) {
    ew_lk++;
    sw_lk++;
  } else if (origin == north && destination == east) {
    wn_lk++;
    we_lk++;
    sn_lk++;
    se_lk++;
    sw_lk++;
    es_lk++;
    ew_lk++;
  } 

  else if (origin == south && destination == north) {
    we_lk++;
    ew_lk++;
    wn_lk++;
    en_lk++;
    es_lk++;
    ne_lk++;

  } else if (origin == south && destination == east) {
    we_lk++;
    ne_lk++;
  } else if (origin == south && destination == west) {
    nw_lk++;
    ns_lk++;
    ne_lk++;
    ew_lk++;
    es_lk++;
    wn_lk++;
    we_lk++;
  }
  else if (origin == west && destination == east) {
    ns_lk++;
    sn_lk++;
    ne_lk++;
    es_lk++;
    se_lk++;
    sw_lk++;

  } else if (origin == west && destination == south) {
    ns_lk++;
    es_lk++;
  } else if (origin == west && destination == north) {
    ns_lk++;
    ne_lk++;
    ew_lk++;
    es_lk++;
    en_lk++;
    sn_lk++;
    sw_lk++;
  }
  else if (origin == east && destination == west) {
    ns_lk++;
    sn_lk++;
    ne_lk++;
    wn_lk++;
    sw_lk++;
    nw_lk++;
  } else if (origin == east && destination == north) {
    sn_lk++;
    wn_lk++;
  } else if (origin == east && destination == south) {
    ns_lk++;
    ne_lk++;
    we_lk++;
    ws_lk++;
    wn_lk++;
    sn_lk++;
    sw_lk++;
  }
}

static void
get_direction(Direction origin, Direction destination) {

  if (origin == north && destination == west) {
    cur_cv = nw;
    cur_lk = nw_lk;
    
  }
  if (origin == north && destination == south) {
    cur_cv = ns;
    cur_lk = ns_lk;
     
  }
  if (origin == north && destination == east) {
    cur_cv = ne;
    cur_lk = ne_lk;

  }
  if (origin == south && destination == north) {
    cur_cv = sn;
    cur_lk = sn_lk;
    
  }
  if (origin == south && destination == west) {
    cur_cv = sw;
    cur_lk = sw_lk;
    
  }
  if (origin == south && destination == east) {
    cur_cv = se;
    cur_lk = se_lk;
    
  }
  if (origin == east && destination == north) {
    cur_cv = en;
    cur_lk = en_lk;
    
  }
  if (origin == east && destination == west) {
    cur_cv = ew;
    cur_lk = ew_lk;
    
  }
  if (origin == east && destination == south) {
    cur_cv = es;
    cur_lk = es_lk;
    
  }
  if (origin == west && destination == north) {
    cur_cv = wn;
    cur_lk = wn_lk;
    
  }  
  if (origin == west && destination == south) {
    cur_cv = ws;
    cur_lk = ws_lk;
    
  }
  if (origin == west && destination == east) {
    cur_cv = we;
    cur_lk = we_lk;
    
  }
}

static void
awake(Direction origin, Direction destination) {
  if (origin == north && destination == south) {
    we_lk--;
    ew_lk--;
    es_lk--;
    ws_lk--;
    wn_lk--;
    sw_lk--;

  } else if (origin == north && destination == west) {
    ew_lk--;
    sw_lk--;
  } else if (origin == north && destination == east) {
    wn_lk--;
    we_lk--;
    sn_lk--;
    se_lk--;
    sw_lk--;
    es_lk--;
    ew_lk--;
  } 

  else if (origin == south && destination == north) {
    we_lk--;
    ew_lk--;
    wn_lk--;
    en_lk--;
    es_lk--;
    ne_lk--;

  } else if (origin == south && destination == east) {
    we_lk--;
    ne_lk--;
  } else if (origin == south && destination == west) {
    nw_lk--;
    ns_lk--;
    ne_lk--;
    ew_lk--;
    es_lk--;
    wn_lk--;
    we_lk--;
  }
  else if (origin == west && destination == east) {
    ns_lk--;
    sn_lk--;
    ne_lk--;
    es_lk--;
    se_lk--;
    sw_lk--;

  } else if (origin == west && destination == south) {
    ns_lk--;
    es_lk--;
  } else if (origin == west && destination == north) {
    ns_lk--;
    ne_lk--;
    ew_lk--;
    es_lk--;
    en_lk--;
    sn_lk--;
    sw_lk--;
  }
  else if (origin == east && destination == west) {
    ns_lk--;
    sn_lk--;
    ne_lk--;
    wn_lk--;
    sw_lk--;
    nw_lk--;
  } else if (origin == east && destination == north) {
    sn_lk--;
    wn_lk--;
  } else if (origin == east && destination == south) {
    ns_lk--;
    ne_lk--;
    we_lk--;
    ws_lk--;
    wn_lk--;
    sn_lk--;
    sw_lk--;
  }
  if (nw_lk == 0) {
    cv_broadcast(nw, intersectionLock);
  }
  if (ns_lk == 0) {
    cv_broadcast(ns, intersectionLock);
  }
  if (ne_lk == 0) {
    cv_broadcast(ne, intersectionLock);
  }
  if (sn_lk == 0) {
    cv_broadcast(sn, intersectionLock);
  }
  if (sw_lk == 0) {
    cv_broadcast(sw, intersectionLock);
  }
  if (se_lk == 0) {
    cv_broadcast(se, intersectionLock);
  }
  if (en_lk == 0) {
    cv_broadcast(en, intersectionLock);
  }
  if (ew_lk == 0) {
    cv_broadcast(ew, intersectionLock);
  }
  if (es_lk == 0) {
    cv_broadcast(es, intersectionLock);
  }
  if (wn_lk == 0) {
    cv_broadcast(wn, intersectionLock);
  }
  if (ws_lk == 0) {
    cv_broadcast(ws, intersectionLock);
  }
  if (we_lk == 0) {
    cv_broadcast(we, intersectionLock);
  }

}



/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  /* replace this default implementation with your own implementation */
  intersectionLock = lock_create("intersectionLock");
  nw = cv_create("nw");
  ns = cv_create("ns");
  ne = cv_create("ne");
  sn = cv_create("sn");
  sw = cv_create("sw");
  se = cv_create("se");
  en = cv_create("en");
  ew = cv_create("ew");
  es = cv_create("es");
  wn = cv_create("wn");
  ws = cv_create("ws");
  we = cv_create("we");

  if (intersectionLock == NULL) {
    panic("could not create lock intersectionLock");
  }
  if (nw == NULL) {
  panic("could not create cv nw");
  }
  if (ns == NULL) {
  panic("could not create cv ns");
  }
  if (ne == NULL) {
  panic("could not create cv ne");
  }
  if (sn == NULL) {
  panic("could not create cv sn");
  }
  if (sw == NULL) {
  panic("could not create cv sw");
  }
  if (se == NULL) {
  panic("could not create cv se");
  }
  if (en == NULL) {
  panic("could not create cv en");
  }
  if (ew == NULL) {
  panic("could not create cv ew");
  }
  if (es == NULL) {
  panic("could not create cv es");
  }
  if (wn == NULL) {
  panic("could not create cv wn");
  }
  if (ws == NULL) {
  panic("could not create cv ws");
  }
  if (we == NULL) {
  panic("could not create cv we");
  }

  nw_lk = false;
  ns_lk = false;
  ne_lk = false;
  sn_lk = false;
  sw_lk = false;
  se_lk = false;
  en_lk = false;
  ew_lk = false;
  es_lk = false;
  wn_lk = false;
  ws_lk = false;
  we_lk = false;

  return;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  /* replace this default implementation with your own implementation */
  KASSERT(intersectionLock != NULL);
  KASSERT(nw != NULL);
  KASSERT(ns != NULL);
  KASSERT(ne != NULL);
  KASSERT(sn != NULL);
  KASSERT(sw != NULL);
  KASSERT(se != NULL);
  KASSERT(en != NULL);
  KASSERT(ew != NULL);
  KASSERT(es != NULL);
  KASSERT(wn != NULL);
  KASSERT(ws != NULL);
  KASSERT(we != NULL);
  cv_destroy(nw);
  cv_destroy(ns);
  cv_destroy(ne);
  cv_destroy(sn);
  cv_destroy(sw);
  cv_destroy(se);
  cv_destroy(en);
  cv_destroy(ew);
  cv_destroy(es);
  cv_destroy(wn);
  cv_destroy(ws);
  cv_destroy(we);
  lock_destroy(intersectionLock);
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{
  lock_acquire(intersectionLock);
  get_direction(origin, destination);
  while (1) {
    if (!cur_lk) {
      locked(origin, destination);
      break;
    } else {
      cv_wait(cur_cv, intersectionLock);
      get_direction(origin, destination);
    }
  }
  lock_release(intersectionLock);

}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
  lock_acquire(intersectionLock);
  awake(origin, destination);
  lock_release(intersectionLock);
  
}
