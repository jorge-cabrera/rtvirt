/******************************************************************************
 * Scheduler for RTVirt based on DP-Wrap
 *
 * By Jorge E. Cabrera
 *
 *******************************************************************************
 *
 * Using "Simple EDF scheduler for xen" as a template
 *
 * by Stephan Diestelhorst (C)  2004 Cambridge University
 * based on code by Mark Williamson (C) 2004 Intel Research Cambridge
 *******************************************************************************/



#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/sched-if.h>
#include <xen/timer.h>
#include <xen/softirq.h>
#include <xen/time.h>
#include <xen/errno.h>

#ifndef NDEBUG
#define CHECK(_p)                                           \
    do {                                                    \
	if ( !(_p) )                                        \
	printk("Check '%s' failed, line %d, file %s\n", \
#_p , __LINE__, __FILE__);               \
    } while ( 0 )
#else
#define CHECK(_p) ((void)0)
#endif

#define SC_INACTIVE	(1)
#define SC_RUNNING	(2)
#define SC_MIGRATING	(4)
#define SC_MIGRATED	(8)
#define SC_ASLEEP	(16)
#define SC_SPLIT	(32)
#define SC_RESET	(64)  // Hint for CPU to reset cputime value
#define SC_SHUTDOWN	(128) // Tell DomU to shutdown
#define SC_DEFAULT	(256) // Hack to allow one-time change of parameters
#define SC_SHIFT	(512) // Hack to shift VCPUs when adjusting BW
#define SC_SPORADIC	(1024) // VCPU is running sporadic task
#define SC_UPDATE_DEADL	(2048) // VCPU is running sporadic task
#define SC_ARRIVED	(4096) // VCPU is running sporadic task
#define SC_WOKEN	(8192) // VCPU is running sporadic task
#define SC_CPU0_BUSY	(16384) // VCPU is running sporadic task

#define EXTRA_QUANTUM (MICROSECS(200))

#define DEBUG_LINES   (50000)

#define DEFAULT_PERIOD (MILLISECS(1000))
#define DEFAULT_SLICE (MILLISECS(150))

extern int sc_debugging;

#define DOM0_PERIOD (MILLISECS(1000))
#define DOM0_SLICE (MILLISECS(1000))

#define PERIOD_MAX MILLISECS(10000) /* 10s  */
#define PERIOD_MIN (MICROSECS(11))  /* 10us */
#define SLICE_MIN (MICROSECS(5))    /*  5us */

#define IMPLY(a, b) (!(a) || (b))
#define EQ(a, b) ((!!(a))== (!!(b)))

#define DEBUG	    	1
//#define DEBUG2	    2
//#define DEBUG3	    3
//#define DEBUG4	    4
//#define DEBUG_ERR   	5

//   if(sc_debugging == 1)   printk (stuff)
//   printk (stuff)
#ifdef DEBUG
#define DPRINTK(stuff...)	\
   printk (stuff)
#else
#define DPRINTK(stuff...)
#endif

#ifdef DEBUG2
#define DPRINTK2(stuff...)	\
   printk (stuff)
#else
#define DPRINTK2(stuff...)
#endif

#ifdef DEBUG3
#define DPRINTK3(stuff...)	\
   printk (stuff)
#else
#define DPRINTK3(stuff...)
#endif

#ifdef DEBUG4
#define DPRINTK4(stuff...)	\
    printk (stuff)
#else
#define DPRINTK4(stuff...)
#endif

#ifdef DEBUG_ERR
#define DPRINTK_ERR(stuff...)	\
	printk (stuff)
#else
#define DPRINTK_ERR(stuff...)
#endif



struct sc_barrier_t {
    atomic_t cpu_count;
    atomic_t updating_global_deadline;
};

struct sc_dom_info {
    struct domain  *domain;
};

struct sc_priv_info {
    /* lock for the whole pluggable scheduler, nests inside cpupool_lock */
    spinlock_t lock;
    struct sc_barrier_t cpu_barrier;
    int       status;
};

struct sc_vcpu_info {
    struct vcpu *vcpu;
    struct list_head list;
    struct list_head d_list;
    struct list_head sc_list;
    /* Parameters for EDF */
    s_time_t  period;  /* = relative deadline */
    s_time_t  slice;   /* = worst case execution time */
    s_time_t  local_deadl;   /* = local deadline */
    s_time_t  local_slice;   /* = worst case local execution time */

    s_time_t  local_deadl_second;   /* = local deadline */
    s_time_t  local_slice_second;   /* = worst case local execution time */

    /* Parameters for migrating DomUs */
    s_time_t  period_a;
    s_time_t  slice_a;

    s_time_t  period_b;
    s_time_t  slice_b;

    int processor_a;
    int processor_b;

    s_time_t  period_new;
    s_time_t  slice_new;

    s_time_t  period_temp;
    s_time_t  slice_temp;

    /* Status of domain */
    int       status;
    int       latency;
    int       weight;
    int       extraweight;
    int       extratime;
    /* Bookkeeping */
    s_time_t  deadl_abs;
    s_time_t  sched_start_abs;
    s_time_t  cputime;
    s_time_t  local_cputime;
    /* Times the domain un-/blocked */
    s_time_t  block_abs;
    s_time_t  unblock_abs;

};

/*	Priority Queue		*/

#define MAX_VCPUs 128

struct heap_node {
    struct sc_vcpu_info * data;
    unsigned long long key;
};

struct heap_node MIN_HEAP[MAX_VCPUs];

/*
 * Initial heap size 0 as heap is empty
 */
int heapsize = 0;

/*
 * Find the parent node
 */
int parent(int i){
    return (i - 1) / 2;
}

void swap(struct heap_node *n1, struct heap_node *n2) {
    struct heap_node temp = *n1 ;
    *n1 = *n2 ;
    *n2 = temp ;
}

/*
 * Use for heap insert and decrease key
 * In if condition check if user attempted to increase value instead of decrease
 * Compare child with parent, if child is smaller then swap
 * Now mark that child and apply the same logic until it reaches root node
 */
void decreaseKey(int i){
    while(i > 0 && MIN_HEAP[parent(i)].key > MIN_HEAP[i].key){
        swap(&MIN_HEAP[i], &MIN_HEAP[parent(i)]);
        i = parent(i);
    }
}

/*
 * Search for a specific node to change its value
 * Returns the index the node being searched
 */
int searchIndex(struct sc_vcpu_info *vcpu){
    for(int i = 0; i < heapsize; ++i){
        if(MIN_HEAP[i].data == vcpu)
            return i;
    }
    printk("--- Oops! BUG in searchIndex ---\n");
    return heapsize;
}

/*
 * Called initially to create a min heap
 * Also called after a node extraction from min heap
 * Since the smallest value node is at the root of min heap
 * After extraction min heapify is called on root node
 * It compares parent with its children
 * If a smaller child found then its swapped with parent and
 * Min heapify is again called on that child to apply same procedure
 */
void minHeapify(int i){
    int largest, r;

    int l = 2*i +1;

    if (l < heapsize && MIN_HEAP[l].key < MIN_HEAP[i].key)
        largest = l;
    else
        largest = i;

    r = 2*i +2;

    if (r < heapsize && MIN_HEAP[r].key < MIN_HEAP[largest].key)
        largest = r;

    if (largest != i){
        swap(&MIN_HEAP[i], &MIN_HEAP[largest]);
        minHeapify(largest);
    }
}

/*
 * Increase heap size to create space for new node
 * Insert the node at that space by calling decrease key function
 */
void heapInsert(struct sc_vcpu_info * vcpu){
    ++heapsize;
    MIN_HEAP[heapsize - 1].key = vcpu->deadl_abs;
    MIN_HEAP[heapsize - 1].data = vcpu;

    decreaseKey(heapsize - 1);
}

/*
 * Heap size less than zero mean no items in heap so nothing to extract
 * For min heap minimum value is at the root which in here is A[0]
 * Save the root and replace root with last element in the heap and decrease the heap size
 * So the root is still in the array in last position but no longer in the heap
 * Since the last element is now the root the heap may be unbalanced
 * So to balance the heap call min heapify on the root node again
 * Lastly return the saved root
 */
void extractMin(void){
    if(heapsize < 0)
        printk("Heap Underflow\n");

    MIN_HEAP[0].key = MIN_HEAP[heapsize - 1].key;
    MIN_HEAP[0].data = MIN_HEAP[heapsize - 1].data;
    --heapsize;
    minHeapify(0);
}

void deleteNode(struct sc_vcpu_info *vcpu){
	int i;

	i = searchIndex(vcpu);
	MIN_HEAP[i].key = 0;
	decreaseKey(i);
	extractMin();
}

void updateMin(struct sc_vcpu_info *vcpu){

	if(MIN_HEAP[0].data != vcpu)
		printk("-- Mismatched data and vcpu -- %d - %llu - %d - %llu \n",
			MIN_HEAP[0].data->vcpu->domain->domain_id,
			MIN_HEAP[0].key,
			vcpu->vcpu->domain->domain_id,
			(unsigned long long ) vcpu->deadl_abs);

	MIN_HEAP[0].key = vcpu->deadl_abs;
	minHeapify(0);
}

/*	END: Priority Queue 	*/
struct vm_debug_entry {
    int domid;
    int vcpuid;
    s_time_t now_time;
    long int ret_time;
    long int slice_time;
    s_time_t alloc;
};

struct sc_cpu_info {
    struct list_head runnableq;
    struct list_head waitq;
    struct list_head inactiveq;
    struct list_head migratedq;
    s_time_t current_slice_expires;
    s_time_t allocated_time;
    unsigned long long hyper_slice;
    unsigned long long hyper_period;
    unsigned long long used_slice;
    unsigned long long used_period;
    unsigned long long new_gl_d;
    int d_array_index;
    int print_index;
    struct vm_debug_entry d_array[DEBUG_LINES];
};

#define SC_PRIV(_ops) \
    ((struct sc_priv_info *)((_ops)->sched_data))
#define EDOM_INFO(d)   ((struct sc_vcpu_info *)((d)->sched_priv))
#define CPU_INFO(cpu)  \
    ((struct sc_cpu_info *)per_cpu(schedule_data, cpu).sched_priv)
#define LIST(d)        (&EDOM_INFO(d)->list)
#define D_LIST(d)      (&EDOM_INFO(d)->d_list)
#define SC_LIST(d)     (&EDOM_INFO(d)->sc_list)
#define RUNQ(cpu)      (&CPU_INFO(cpu)->runnableq)
#define WAITQ(cpu)     (&CPU_INFO(cpu)->waitq)
#define INACTIVEQ(cpu) (&CPU_INFO(cpu)->inactiveq)
#define MIGQ(cpu) (&CPU_INFO(cpu)->migratedq)
#define HSLICE(cpu)    (CPU_INFO(cpu)->hyper_slice)
#define HPERIOD(cpu)   (CPU_INFO(cpu)->hyper_period)
#define USEDSLICE(cpu)    (CPU_INFO(cpu)->used_slice)
#define USEDPERIOD(cpu)   (CPU_INFO(cpu)->used_period)
#define IDLETASK(cpu)  (idle_vcpu[cpu])

#define PERIOD_BEGIN(inf) ((inf)->deadl_abs - (inf)->period)

#define DIV_UP(x,y) (((x) + (y) - 1) / y)

#define sc_runnable(edom)  (!(EDOM_INFO(edom)->status & SC_ASLEEP))
//#define sc_active(edom)  (!(EDOM_INFO(edom)->status & SC_INACTIVE))

static void init_sc_barrier(struct sc_barrier_t* b)
{
    atomic_set(&b->cpu_count, 0);
    atomic_set(&b->updating_global_deadline, -1);
}

static int last_assigned_pcpu = 0;
static int dom0_cpu_count = 0;

/* return the greatest common divisor of a and b using Euclid's algorithm,
   modified to be fast when one argument much greater than the other, and
   coded to avoid unnecessary swapping */
static unsigned long long gcd(unsigned long long a, unsigned long long b)
{
    unsigned long long c;

    while (a && b)
	if (a > b) {
	    c = b;
	    while (a - c >= c)
		c <<= 1;
	    a -= c;
	}
	else {
	    c = a;
	    while (b - c >= c)
		c <<= 1;
	    b -= c;
	}
    return a + b;
}

// FIXME: Currently this function does not handle overflow events!!!!
unsigned long long lcm(unsigned long long a, unsigned long long b)
{
    if (a && b)
	return (a * b) / gcd(a, b);
    else if (b)
	return b;

    return a;
}

static void sc_dump_cpu_state(const struct scheduler *ops, int i);

static inline int __task_on_sclist(struct vcpu *d)
{
    return (((SC_LIST(d))->next != NULL) && (SC_LIST(d)->next != SC_LIST(d)));
}

static inline int __task_on_queue(struct vcpu *d)
{
    return (((LIST(d))->next != NULL) && (LIST(d)->next != LIST(d)));
}

static inline int task_on_deadline_queue(struct vcpu *d)
{
    return (((D_LIST(d))->next != NULL) && (D_LIST(d)->next != D_LIST(d)));
}

static inline void __del_from_queue(struct vcpu *d)
{
    struct list_head *list = LIST(d);
    ASSERT(__task_on_queue(d));
    list_del(list);
    list->next = NULL;
    ASSERT(!__task_on_queue(d));
}

typedef int(*list_comparer)(struct list_head* el1, struct list_head* el2);

static inline void list_insert_sort(
    struct list_head *list, struct list_head *element, list_comparer comp)
{
    struct list_head     *cur;

    /* Iterate through all elements to find our "hole" */
    list_for_each( cur, list )
        if ( comp(element, cur) < 0 )
            break;

    /* cur now contains the element, before which we'll enqueue */
    list_add(element, cur->prev);
}

#define DOMAIN_COMPARER(name, field, comp1, comp2)                      \
static int name##_comp(struct list_head* el1, struct list_head* el2)    \
{                                                                       \
    struct sc_vcpu_info *d1, *d2;                                     \
    d1 = list_entry(el1,struct sc_vcpu_info, field);                  \
    d2 = list_entry(el2,struct sc_vcpu_info, field);                  \
    if ( (comp1) == (comp2) )                                           \
        return 0;                                                       \
    if ( (comp1) < (comp2) )                                            \
        return -1;                                                      \
    else                                                                \
        return 1;                                                       \
}

DOMAIN_COMPARER(runq, d_list, d1->deadl_abs, d2->deadl_abs);

static struct list_head sc_list_head;

static int reverse_order_next = 1; // This variable should only be accessed by CPU 0
static s_time_t global_deadline = 0;

// A periodic VCPU always has its BW reservation activated.
// A sporadic VCPU activates it only when it arrives.
static void activate_cpu_bw_reservation(struct vcpu *d)
{
    int first_cpu, second_cpu;

    first_cpu = EDOM_INFO(d)->processor_a;
    second_cpu = first_cpu + 1;
/*
    DPRINTK4("------ CPU: %d - FIRST: %d - ID: %6d.%d - %s ------\n",
	    smp_processor_id(),
	    first_cpu,
	    d->domain->domain_id,
	    d->vcpu_id,
	    __func__);
*/
    if(!vcpu_runnable(d) || EDOM_INFO(d)->status & SC_WOKEN)
    {
	d->processor = first_cpu;
	list_move_tail(LIST(d), WAITQ(first_cpu));
	return;
    }

    EDOM_INFO(d)->status |= SC_WOKEN;

    if((CPU_INFO(first_cpu)->used_slice + 1000)
	    > CPU_INFO(first_cpu)->used_period)
    {
	CPU_INFO(first_cpu)->used_slice =
	    CPU_INFO(first_cpu)->used_period;
    }

    if((CPU_INFO(first_cpu)->used_slice + EDOM_INFO(d)->slice_new)
	    < CPU_INFO(first_cpu)->used_period)
    {
	CPU_INFO(first_cpu)->used_slice =
	    CPU_INFO(first_cpu)->used_slice +
	    EDOM_INFO(d)->slice_new;

	if(d->processor != first_cpu)
	{
	    d->processor = first_cpu;
	    list_move_tail(LIST(d), WAITQ(first_cpu));
	    //cpu_raise_softirq(d->processor, SCHEDULE_SOFTIRQ);
	}
	else
	{
	    d->processor = first_cpu;
	    list_move_tail(LIST(d), WAITQ(first_cpu));
	}
    }
    else if((CPU_INFO(first_cpu)->used_slice + EDOM_INFO(d)->slice_new)
	    == CPU_INFO(first_cpu)->used_period)
    {
	CPU_INFO(first_cpu)->used_slice =
	    CPU_INFO(first_cpu)->used_period;

	if(d->processor != first_cpu)
	{
	    d->processor = first_cpu;
	    list_move_tail(LIST(d), WAITQ(first_cpu));
	    //cpu_raise_softirq(d->processor, SCHEDULE_SOFTIRQ);
	}
	else
	{
	    d->processor = first_cpu;
	    list_move_tail(LIST(d), WAITQ(first_cpu));
	}
    }
    else if(CPU_INFO(first_cpu)->used_slice
	    == CPU_INFO(first_cpu)->used_period)
    {
	CPU_INFO(second_cpu)->used_slice =
	    CPU_INFO(second_cpu)->used_slice +
	    EDOM_INFO(d)->slice_new;

	//TODO: Check if need to migrate it
	if(d->processor != second_cpu)
	{
	    d->processor = second_cpu;
	    list_move_tail(LIST(d), WAITQ(second_cpu));
	    //cpu_raise_softirq(d->processor, SCHEDULE_SOFTIRQ);
	}
	else
	{
	    d->processor = second_cpu;
	    list_move_tail(LIST(d), WAITQ(second_cpu));
	}
    }
    else
    {
	EDOM_INFO(d)->status |= SC_SPLIT;
	EDOM_INFO(d)->status |= SC_MIGRATING;

	EDOM_INFO(d)->slice_a =
	    CPU_INFO(first_cpu)->used_period -
	    CPU_INFO(first_cpu)->used_slice;

	EDOM_INFO(d)->slice_b =
	    EDOM_INFO(d)->slice_new -
	    EDOM_INFO(d)->slice_a;

	EDOM_INFO(d)->period_a =
	    EDOM_INFO(d)->period_b = 100000;

	CPU_INFO(first_cpu)->used_slice =
	    CPU_INFO(first_cpu)->used_period;

	CPU_INFO(second_cpu)->used_slice =
	    EDOM_INFO(d)->slice_b;

	EDOM_INFO(d)->processor_b = second_cpu;

	if(d->processor != second_cpu)
	{
	    d->processor = second_cpu;
	    list_move_tail(LIST(d), INACTIVEQ(second_cpu));
	    //-->cpu_raise_softirq(d->processor, SCHEDULE_SOFTIRQ);
	}
	else
	{
	    d->processor = second_cpu;
	    list_move_tail(LIST(d), INACTIVEQ(second_cpu));
	}
    }
}

static void set_cpu_bw_reservation(struct vcpu *d)
{
    int first_cpu, second_cpu;

    /*
    DPRINTK4("------ CPU: %d - ID: %6d.%d - %s ------\n",
	    smp_processor_id(),
	    d->domain->domain_id,
	    d->vcpu_id,
	    __func__);
*/

    first_cpu = EDOM_INFO(d)->processor_a;
    second_cpu = EDOM_INFO(d)->processor_a + 1;

    if(EDOM_INFO(d)->status & SC_SPORADIC || EDOM_INFO(d)->status & SC_ARRIVED)
    {
	EDOM_INFO(d)->status &= ~SC_SPLIT;
	EDOM_INFO(d)->status &=	~SC_MIGRATING;

	activate_cpu_bw_reservation(d);
    }
    else
    {
	if(EDOM_INFO(d)->status & SC_SPLIT)
	{
	    CPU_INFO(first_cpu)->used_slice =
		CPU_INFO(first_cpu)->used_slice +
		EDOM_INFO(d)->slice_a;

	    CPU_INFO(second_cpu)->used_slice =
		CPU_INFO(second_cpu)->used_slice +
		EDOM_INFO(d)->slice_b;

	    CPU_INFO(first_cpu)->used_period =
		CPU_INFO(second_cpu)->used_period = 100000;

	    // HACK: The split VCPU which is periodic must be placed
	    // back into the second_cpu's runq from the first_cpu's runq

	    d->processor = second_cpu;
	    list_move_tail(LIST(d), INACTIVEQ(second_cpu));
	}
	else
	{
	    CPU_INFO(d->processor)->used_slice =
		CPU_INFO(d->processor)->used_slice +
		EDOM_INFO(d)->slice_new;
	    CPU_INFO(d->processor)->used_period = 100000;
	}
    }
}

// A periodic VCPU always has its BW reservation activated.
// A sporadic VCPU activates it only when it arrives.
static void dynamic_activate(struct vcpu *d)
{
    int first_cpu, second_cpu;
    DPRINTK3("------ CPU: %d - ID: %6d.%d - %s ------\n",
	    smp_processor_id(),
	    d->domain->domain_id,
	    d->vcpu_id,
	    __func__);

    //if(!(EDOM_INFO(d)->status & SC_ARRIVED))
    if(!vcpu_runnable(d) || EDOM_INFO(d)->status & SC_WOKEN)
	return;

    EDOM_INFO(d)->status |= SC_WOKEN;

    first_cpu = EDOM_INFO(d)->processor_a;
    second_cpu = first_cpu + 1;

    if((CPU_INFO(first_cpu)->used_slice + 1000)
	    > CPU_INFO(first_cpu)->used_period)
    {
	CPU_INFO(first_cpu)->used_slice =
	    CPU_INFO(first_cpu)->used_period;
    }

    if((CPU_INFO(first_cpu)->used_slice + EDOM_INFO(d)->slice_new)
	    < CPU_INFO(first_cpu)->used_period)
    {
	CPU_INFO(first_cpu)->used_slice =
	    CPU_INFO(first_cpu)->used_slice +
	    EDOM_INFO(d)->slice_new;
    }
    else if((CPU_INFO(first_cpu)->used_slice + EDOM_INFO(d)->slice_new)
	    == CPU_INFO(first_cpu)->used_period)
    {
	CPU_INFO(first_cpu)->used_slice =
	    CPU_INFO(first_cpu)->used_period;
    }
    else if(CPU_INFO(first_cpu)->used_slice
	    == CPU_INFO(first_cpu)->used_period)
    {
	CPU_INFO(second_cpu)->used_slice =
	    CPU_INFO(second_cpu)->used_slice +
	    EDOM_INFO(d)->slice_new;

	//TODO: Check if need to migrate it
	if(d->processor != second_cpu)
	{
	    d->processor = second_cpu;
	    //list_move_tail(LIST(d), INACTIVEQ(second_cpu));
	    //cpu_raise_softirq(d->processor, SCHEDULE_SOFTIRQ);
	}
	/*else
	{
	    d->processor = second_cpu;
	    list_move_tail(LIST(d), INACTIVEQ(second_cpu));
	}*/
    }
    else
    {
	EDOM_INFO(d)->status |= SC_SPLIT;
	EDOM_INFO(d)->status |= SC_MIGRATING;

	EDOM_INFO(d)->slice_a =
	    CPU_INFO(first_cpu)->used_period -
	    CPU_INFO(first_cpu)->used_slice;

	EDOM_INFO(d)->slice_b =
	    EDOM_INFO(d)->slice_new -
	    EDOM_INFO(d)->slice_a;

	EDOM_INFO(d)->period_a =
	    EDOM_INFO(d)->period_b = 100000;

	CPU_INFO(first_cpu)->used_slice =
	    CPU_INFO(first_cpu)->used_period;

	CPU_INFO(second_cpu)->used_slice =
	    EDOM_INFO(d)->slice_b;

	EDOM_INFO(d)->processor_b = second_cpu;

	if(d->processor != second_cpu)
	{
	    d->processor = second_cpu;
	    //list_move_tail(LIST(d), INACTIVEQ(second_cpu));
	    //cpu_raise_softirq(d->processor, SCHEDULE_SOFTIRQ);
	}
	/*else
	{
	    d->processor = second_cpu;
	    list_move_tail(LIST(d), INACTIVEQ(second_cpu));
	}*/
    }
}

static void dynamic_reservation(struct vcpu *d)
{
    int first_cpu, second_cpu;

    DPRINTK3("------ CPU: %d - ID: %6d.%d - %s ------\n",
	    smp_processor_id(),
	    d->domain->domain_id,
	    d->vcpu_id,
	    __func__);

    first_cpu = EDOM_INFO(d)->processor_a;
    second_cpu = EDOM_INFO(d)->processor_a + 1;

    if(EDOM_INFO(d)->status & SC_SPORADIC || EDOM_INFO(d)->status & SC_ARRIVED)
    {
	EDOM_INFO(d)->status &= ~SC_SPLIT;
	EDOM_INFO(d)->status &=	~SC_MIGRATING;

	dynamic_activate(d);
    }
}

static int dp_wrap_assign_pcpu(struct vcpu *v, const struct scheduler *ops)
{
    // -> struct sc_priv_info *prv = SC_PRIV(ops);
    // -> unsigned long flags;

    int cpu_i;
    spinlock_t *lock;
    s_time_t hslice_total, hperiod_total;
    s_time_t hslice, hremainder, vslice;
    unsigned int nr_cpus = cpumask_last(&cpu_online_map) + 1;

    // -> spin_lock_irqsave(&prv->lock, flags);

    EDOM_INFO(v)->status &= ~SC_SHIFT;
    EDOM_INFO(v)->status &= ~SC_SPLIT;
    EDOM_INFO(v)->status &= ~SC_MIGRATED;

    DPRINTK("------ CPU: %d - %s - %d ------\n",
	    smp_processor_id(),
	    __func__,
	    __LINE__);


    //printk("--- period new: %llu - slice new: %llu ---\n",
//	    (unsigned long long) EDOM_INFO(v)->period_new,
//	    (unsigned long long) EDOM_INFO(v)->slice_new);


//    this_cpu = smp_processor_id();
    for(cpu_i = 0; cpu_i < nr_cpus; cpu_i++)
    {
	lock = NULL;

//	if(this_cpu != cpu_i)
//	    while(!lock)
//		lock = pcpu_schedule_trylock(cpu_i);

	if( HSLICE(cpu_i) == HPERIOD(cpu_i) )
	{
//	    if(this_cpu != cpu_i)
//		pcpu_schedule_unlock(lock, cpu_i);
	    continue;
	}
	else if( HSLICE(cpu_i) != 0 && HSLICE(cpu_i) + 1000 >= HPERIOD(cpu_i) )
	{
	    HSLICE(cpu_i) = HPERIOD(cpu_i) = 100000;
	    continue;
	}

	DPRINTK("-- Check1 - cpu: %d - s: %llu p: %llu --\n",
		cpu_i, HSLICE(cpu_i), HPERIOD(cpu_i));

	hperiod_total = lcm( HPERIOD(cpu_i), EDOM_INFO(v)->period_new);

	hslice = ((HSLICE(cpu_i) * (hperiod_total/HPERIOD(cpu_i))));
	vslice =  (EDOM_INFO(v)->slice_new * (hperiod_total/EDOM_INFO(v)->period_new));
	hremainder = hperiod_total - hslice;

	hslice_total = ((HSLICE(cpu_i) * (hperiod_total/HPERIOD(cpu_i)))) +
	    (EDOM_INFO(v)->slice_new * (hperiod_total/EDOM_INFO(v)->period_new));

	if(hslice_total < hperiod_total)
	{
	    HSLICE(cpu_i) = hslice_total;
	    HPERIOD(cpu_i) = hperiod_total;

	    if(v->processor != cpu_i)
	    {
		v->processor = cpu_i;
		EDOM_INFO(v)->processor_a = cpu_i;
//		EDOM_INFO(v)->status |= SC_MIGRATED;
		last_assigned_pcpu = (cpu_i > last_assigned_pcpu ? cpu_i : last_assigned_pcpu);
		//set_bit(_VPF_migrating, &v->pause_flags);
		list_move_tail(LIST(v), INACTIVEQ(cpu_i));
		cpu_raise_softirq(v->processor, SCHEDULE_SOFTIRQ);
	    }
	    else
	    {
		v->processor = cpu_i;
		EDOM_INFO(v)->processor_a = cpu_i;
		list_move_tail(LIST(v), INACTIVEQ(cpu_i));
	    }

	}
	else
	{
	    if(hslice_total > hperiod_total)
	    {
		if(cpu_i + 1 == nr_cpus)
		{
//		    if(this_cpu != cpu_i)
//			pcpu_schedule_unlock(lock, cpu_i);
		    // -> spin_unlock_irqrestore(&prv->lock, flags);
		    return 0;
		}

		// ->processor point to the host processor, ->processor_a is the processor which schedules
		HSLICE(cpu_i) = 100000;
		HPERIOD(cpu_i) = 100000;

/*		if(v->processor != cpu_i)
		{
		    v->processor = cpu_i;
		    list_move_tail(LIST(v), INACTIVEQ(cpu_i));
		    cpu_raise_softirq(v->processor, SCHEDULE_SOFTIRQ);
		}
*/
		EDOM_INFO(v)->processor_a = cpu_i;

		EDOM_INFO(v)->period_a = hperiod_total;
		EDOM_INFO(v)->slice_a = hremainder; //FIXME: Hack to avoid overflows

//		if(this_cpu != cpu_i)
//		    pcpu_schedule_unlock(lock, cpu_i);

		if(HSLICE(cpu_i) > HPERIOD(cpu_i))
		    printk("-- NOOP - Something bad happened: cpu: %d - s: %llu p: %llu --\n",
			    cpu_i, HSLICE(cpu_i), HPERIOD(cpu_i));

		cpu_i++;
		last_assigned_pcpu = (cpu_i > last_assigned_pcpu ? cpu_i : last_assigned_pcpu);

//		lock = NULL;

//		if(this_cpu != cpu_i)
//		    while(!lock)
//			lock = pcpu_schedule_trylock(cpu_i);

		if(HSLICE(cpu_i) > HPERIOD(cpu_i))
		    printk("-- NOOP - Something bad happened: cpu: %d - s: %llu p: %llu --\n", cpu_i, HSLICE(cpu_i), HPERIOD(cpu_i));

		EDOM_INFO(v)->slice_b = HSLICE(cpu_i) = (vslice - hremainder);
		EDOM_INFO(v)->period_b = HPERIOD(cpu_i) = hperiod_total;
		EDOM_INFO(v)->processor_b = cpu_i;
		EDOM_INFO(v)->status |= SC_SPLIT;

//		if(reverse_order_next > 0)
//		    cpu_i--;

		if(v->processor != cpu_i)
		{
		    v->processor = cpu_i;
//		    EDOM_INFO(v)->status |= SC_MIGRATED;
		    //set_bit(_VPF_migrating, &v->pause_flags);
		    list_move_tail(LIST(v), INACTIVEQ(cpu_i));

		    if(CPU_INFO(cpu_i)->new_gl_d == 0)
			CPU_INFO(cpu_i)->new_gl_d = global_deadline;

		    cpu_raise_softirq(v->processor, SCHEDULE_SOFTIRQ);
		}
		else
		{
		    v->processor = cpu_i;
		    list_move_tail(LIST(v), INACTIVEQ(cpu_i));
		}

	    }
	    else
	    {
		HSLICE(cpu_i) = 100000;
		HPERIOD(cpu_i) = 100000;

		if(v->processor != cpu_i)
		{
		    v->processor = cpu_i;
		    EDOM_INFO(v)->processor_a = cpu_i;
//		    EDOM_INFO(v)->status |= SC_MIGRATED;
		    last_assigned_pcpu = (cpu_i > last_assigned_pcpu ? cpu_i : last_assigned_pcpu);
		    //set_bit(_VPF_migrating, &v->pause_flags);
		    list_move_tail(LIST(v), INACTIVEQ(cpu_i));
		    cpu_raise_softirq(v->processor, SCHEDULE_SOFTIRQ);
		}
		else
		{
		    v->processor = cpu_i;
		    EDOM_INFO(v)->processor_a = cpu_i;
		    list_move_tail(LIST(v), INACTIVEQ(cpu_i));
		}
	    }
	}

	if(EDOM_INFO(v)->status & SC_SPLIT)
	{
	    DPRINTK("-- Check2 - CPU: %d - ID:%d.%d - cpu1: %d - cpu2: %d - slice_a: %lld - period_a: %lld - slice_b: %lld: - period_b: %lld --\n",
		    smp_processor_id(),
		    v->domain->domain_id,
		    v->vcpu_id,
		    EDOM_INFO(v)->processor_a,
		    EDOM_INFO(v)->processor_b,
		    (long long) EDOM_INFO(v)->slice_a,
		    (long long) EDOM_INFO(v)->period_a,
		    (long long) EDOM_INFO(v)->slice_b,
		    (long long) EDOM_INFO(v)->period_b);
	}
	else
	{
	    DPRINTK("-- Check2 - CPU: %d - ID:%d.%d - cpu1: %d - slice_a: %lld - period_a: %lld --\n",
		    smp_processor_id(),
		    v->domain->domain_id,
		    v->vcpu_id,
		    v->processor,
		    (long long) EDOM_INFO(v)->slice_new,
		    (long long) EDOM_INFO(v)->period_new);
	}

//	if(this_cpu != cpu_i)
//	    pcpu_schedule_unlock(lock, cpu_i);

	// -> spin_unlock_irqrestore(&prv->lock, flags);
	return 1;
    }
    // -> spin_unlock_irqrestore(&prv->lock, flags);
    return 0;
}

static struct list_head deadline_queue; // FIXME: This creates one for each CPU. It should be one global one!

static void tell_vcpus_to_find_new_pcpus(struct vcpu *v, struct sc_barrier_t* b, const struct scheduler *ops)
{
    //struct sc_vcpu_info *curinf;
   // struct list_head     *cur, *tmp;
    struct sc_priv_info *prv = SC_PRIV(ops);

    printk("------ CPU: %d - %s - %d ------\n", smp_processor_id(), __func__, __LINE__);

    if(prv->status & SC_SHIFT)
	return;

    prv->status |= SC_SHIFT;

    //curinf = list_entry(sc_list_head.prev, struct sc_vcpu_info, sc_list);
    //atomic_set(&b->cpu_count, last_assigned_pcpu);

}

/*
 * FIXME: We don't do error checking yet, to ensure that we have bandwidth
 * left.
 */

static void sc_insert_vcpu(const struct scheduler *ops, struct vcpu *v)
{
    DPRINTK("------ CPU: %d - ID: %6d.%d - %s ------\n",
	    smp_processor_id(),
	    v->domain->domain_id,
	    v->vcpu_id,
	    __func__);
    //FIXME: For some reason Xen creates domains 1 through 7 and then destroys these
    // This messes up my PCPU assignment function.
    //if ( is_idle_vcpu(v) || (v->domain->domain_id > 0 && v->domain->domain_id < 8) )


    if ( is_idle_vcpu(v) )
	v->processor = v->vcpu_id;
    else if(!(EDOM_INFO(v)->status & SC_SHUTDOWN))
    {
	if(v->domain->domain_id == 0)
	    dom0_cpu_count++;
	v->processor = 0;
	dp_wrap_assign_pcpu(v, ops);
    }

    if ( is_idle_vcpu(v) )
    {
	v->processor = v->vcpu_id;
	EDOM_INFO(v)->deadl_abs = 0;
	EDOM_INFO(v)->status &= ~SC_ASLEEP;
    }
}

static void sc_remove_vcpu(const struct scheduler *ops, struct vcpu *v)
{
    struct list_head *list;
    struct sc_vcpu_info *inf     = EDOM_INFO(v);

    DPRINTK("------ CPU: %d - %s ------\n",
	    smp_processor_id(),
	    __func__);

    inf->status |= SC_SHUTDOWN;

    list = D_LIST(v);
    list_del(list);

    //deleteNode(inf);
    list = LIST(v);
    list_del(list);

    list = SC_LIST(v);
    list_del(list);
}

static void *sc_alloc_vdata(const struct scheduler *ops, struct vcpu *v, void *dd)
{
    struct sc_vcpu_info *inf;

    DPRINTK("------ CPU: %d - ID: %6d.%d - %s ------\n",
	    smp_processor_id(),
	    v->domain->domain_id,
	    v->vcpu_id,
	    __func__);

    inf = xzalloc(struct sc_vcpu_info);
    if ( inf == NULL )
	return NULL;


    inf->vcpu = v;

    inf->local_cputime = 0;
    inf->local_deadl = 0;
    inf->deadl_abs   = 0;
    inf->status      = SC_ASLEEP | SC_INACTIVE;
    inf->extraweight = 0;
    inf->weight = 0;
    inf->latency     = 0;


    if(v->domain->domain_id == 0)
    {
	inf->period      = DOM0_PERIOD;
	inf->slice       = DOM0_SLICE;
    }
    else
    {
	inf->period      = DEFAULT_PERIOD;
	inf->slice       = DEFAULT_SLICE;
	inf->status  |= SC_SPORADIC;

	if(v->vcpu_id == 0)
	    inf->status  |= SC_DEFAULT;
    }

    inf->period_temp = inf->period / 1000;
    inf->slice_temp = inf->slice / 1000;

    inf->period_new = inf->period_temp;
    inf->slice_new = inf->slice_temp;

    inf->slice_new = (100000 * inf->slice_new) / inf->period_new;
    inf->period_new = 100000;

    INIT_LIST_HEAD(&(inf->list));
    INIT_LIST_HEAD(&(inf->d_list));
    INIT_LIST_HEAD(&(inf->sc_list));

    return inf;
}

    static void *
sc_alloc_pdata(const struct scheduler *ops, int cpu)
{
    struct sc_cpu_info *spc;

    DPRINTK("------ CPU: %d - %s ------\n",
	    smp_processor_id(),
	    __func__);

    spc = xzalloc(struct sc_cpu_info);
    BUG_ON(spc == NULL);
    INIT_LIST_HEAD(&spc->runnableq);
    INIT_LIST_HEAD(&spc->waitq);
    INIT_LIST_HEAD(&spc->inactiveq);
    INIT_LIST_HEAD(&spc->migratedq);
    spc->hyper_slice = 0;
    spc->hyper_period = 100000;
    spc->new_gl_d = 0;
    spc->d_array_index = 0;
    spc->print_index = 0;
    spc->current_slice_expires = 0;
    spc->allocated_time = 0;

    spc->used_slice = 0;
    spc->used_period = 10000;

    return (void *)spc;
}

    static void
sc_free_pdata(const struct scheduler *ops, void *spc, int cpu)
{
    DPRINTK("------ CPU: %d - %s ------\n",
	    smp_processor_id(),
	    __func__);

    if ( spc == NULL )
	return;

    xfree(spc);
}

static void sc_free_vdata(const struct scheduler *ops, void *priv)
{
    DPRINTK("------ CPU: %d - %s ------\n",
	    smp_processor_id(),
	    __func__);

    xfree(priv);
}

    static void *
sc_alloc_domdata(const struct scheduler *ops, struct domain *d)
{
    DPRINTK("------ CPU: %d - %s ------\n",
	    smp_processor_id(),
	    __func__);

    return xzalloc(struct sc_dom_info);
}

static int sc_init_domain(const struct scheduler *ops, struct domain *d)
{
    DPRINTK("------ CPU: %d - %s ------\n",
	    smp_processor_id(),
	    __func__);

    d->sched_priv = sc_alloc_domdata(ops, d);
    if ( d->sched_priv == NULL )
	return -ENOMEM;


    return 0;
}

static void sc_free_domdata(const struct scheduler *ops, void *data)
{
    DPRINTK("------ CPU: %d - %s ------\n",
	    smp_processor_id(),
	    __func__);

    xfree(data);
}

static void sc_destroy_domain(const struct scheduler *ops, struct domain *d)
{
    struct sc_priv_info *prv = SC_PRIV(ops);

    DPRINTK("------ CPU: %d - %s ------\n",
	    smp_processor_id(),
	    __func__);

    tell_vcpus_to_find_new_pcpus(NULL, &prv->cpu_barrier, ops);

    sc_free_domdata(ops, d->sched_priv);
}

static int sc_pick_cpu(const struct scheduler *ops, struct vcpu *v)
{
    DPRINTK("------ CPU: %d - ID: %6d.%d - %s - %d ------\n",
	    smp_processor_id(),
	    v->domain->domain_id,
	    v->vcpu_id,
	    __func__,
	    v->processor);

    /*
    if ( is_idle_vcpu(v) )
    {
	cpumask_t online_affinity;
	cpumask_t *online;

	online = cpupool_scheduler_cpumask(v->domain->cpupool);
	cpumask_and(&online_affinity, v->cpu_hard_affinity, online);
	return cpumask_cycle(v->vcpu_id % cpumask_weight(&online_affinity) - 1,
		&online_affinity);
    }
    */
    if ( is_idle_vcpu(v) )
	return v->vcpu_id;

    //return 0;
    return v->processor;
}

static s_time_t global_slice_start = 0;

static int sc_init(struct scheduler *ops)
{
    struct sc_priv_info *prv;

    DPRINTK("------ CPU: %d - %s ------\n",
	    smp_processor_id(),
	    __func__);

    prv = xzalloc(struct sc_priv_info);
    if ( prv == NULL )
	return -ENOMEM;

    ops->sched_data = prv;
    spin_lock_init(&prv->lock);
    init_sc_barrier(&prv->cpu_barrier);
    prv->status = 0;
    INIT_LIST_HEAD(&deadline_queue);
    INIT_LIST_HEAD(&sc_list_head);
    sc_debugging = 4;

    return 0;
}

static void sc_deinit(const struct scheduler *ops)
{
    struct sc_priv_info *prv;

    DPRINTK("------ CPU: %d - %s ------\n",
	    smp_processor_id(),
	    __func__);

    prv = SC_PRIV(ops);
    if ( prv != NULL )
	xfree(prv);
}
/*
static s_time_t get_last_local_deadl(struct sc_vcpu_info *inf)
{
    if(inf->status & SC_SPLIT)
    {
	if(inf->processor_a == smp_processor_id())
	    return inf->local_deadl_second;
	else
	    return inf->local_deadl;
    }

    return inf->local_deadl;
}

static s_time_t get_total_local_slice(struct sc_vcpu_info *inf)
{
    if(inf->status & SC_SPLIT)
	    return inf->local_slice_second + inf->local_slice;

    return inf->local_slice;
}
*/

static s_time_t get_local_slice(struct sc_vcpu_info *inf)
{
    if(inf->status & SC_SPLIT)
    {
	if(inf->processor_a == smp_processor_id())
	    return inf->local_slice;
	else
	    return inf->local_slice_second;
    }

    return inf->local_slice;
}

static s_time_t get_local_deadl(struct sc_vcpu_info *inf)
{
    if(inf->status & SC_SPLIT)
    {
	if(inf->processor_a == smp_processor_id())
	    return inf->local_deadl;
	else
	    return inf->local_deadl_second;
    }

    return inf->local_deadl;
}


static int sc_active(struct sc_vcpu_info *inf, s_time_t now)
{
//    if( !(inf->status & SC_INACTIVE) && ((get_local_deadl(inf) - get_local_slice(inf)) < now )  )
    //if( !(inf->status & SC_INACTIVE) && ((get_local_deadl(inf) - get_local_slice(inf)) < (now) )  )
    if( !(inf->status & SC_INACTIVE) )
	return 1;

    /*
    printk("--- D --- Now: %ld - Local_Deadl: %ld - Local_slice: %ld - Diff_local: %ld - Diff_now: %ld ---\n",
	    now,
	    get_local_deadl(inf),
	    get_local_slice(inf),
	    get_local_deadl(inf) - get_local_slice(inf),
	    (get_local_deadl(inf) - get_local_slice(inf)) - now);
    if( !(inf->status & SC_INACTIVE) )
	return 1;
*/

    return 0;
}


static void calculate_new_local_deadlines(int cpu, s_time_t now, const struct scheduler *ops)
{
    //struct list_head     *migq     = MIGQ(cpu);
    int array_index;
    struct list_head     *runq     = RUNQ(cpu);
    struct list_head     *waitq     = WAITQ(cpu);
    struct list_head     *inactiveq = INACTIVEQ(cpu);
    struct list_head     *cur, *tmp;
    struct sc_vcpu_info *curinf, *first;
    s_time_t slice_length = global_deadline - (global_slice_start);
    //s_time_t              new_now = NOW();
    //s_time_t slice_length = global_deadline - now;
    s_time_t prev, curr;
    struct shared_info *si;
    int loop_detection = 0;

    if(sc_debugging == 1)
    {
	if(CPU_INFO(cpu)->d_array_index < DEBUG_LINES)
	{
	    array_index = CPU_INFO(cpu)->d_array_index;
	    CPU_INFO(cpu)->d_array[array_index].domid = 0;
	    CPU_INFO(cpu)->d_array[array_index].vcpuid = 0;
	    CPU_INFO(cpu)->d_array[array_index].now_time = now;
	    CPU_INFO(cpu)->d_array[array_index].ret_time = 0;
	    CPU_INFO(cpu)->d_array[array_index].slice_time = slice_length;
	    CPU_INFO(cpu)->d_array[array_index].alloc = CPU_INFO(cpu)->allocated_time;
	    CPU_INFO(cpu)->allocated_time = 0;
	    CPU_INFO(cpu)->d_array_index++;
	}
    }

    /*
    if(sc_debugging == 1)
    {
	printk("-- STA --- CPU: %d ------\n",
		cpu);
    }
*/
/*
    if(!list_empty(migq))
    {
	list_for_each_safe ( cur, tmp, migq )
	{
	    curinf = list_entry(cur,struct sc_vcpu_info,list);

	    //if(!(curinf->status & SC_MIGRATED && get_local_deadl(curinf) > now))
	    //if(get_local_deadl(curinf) <= now)
	    //{
		curinf->status &= ~SC_MIGRATED;
		printk("--- OH OH --- Something is wrong! RUNQ: %d - should be empty but it's not: ID: %6d.%d - Remaining: %ld -\n",
			cpu,
			curinf->vcpu->domain->domain_id,
			curinf->vcpu->vcpu_id,
			now - get_local_deadl(curinf));
		list_move(LIST(curinf->vcpu), waitq);
	    //}
	}
    }
*/

    list_for_each_safe ( cur, tmp, waitq )
    {
	if(loop_detection++ > 20)
	    printk("**** OOPS: Caught in an infinite loop: %d *****\n", __LINE__);

	curinf = list_entry(cur, struct sc_vcpu_info, list);


	// IDEALLY, we want periodcs, followed by sporadic arrived,
	// sporadic runnable. Instead, of creating a new third list, or
	// doing to passes to runq, I just use inactiveq as a third list,
	// which will contain the sporadic runnables (not active), and then
	// these are moved to the end of runq.

	if(curinf->status & SC_ARRIVED)
	{
	    curinf->status &= ~SC_ARRIVED;
	    curinf->status |= SC_SPORADIC;
	}

	if(curinf->status & SC_SPORADIC)
	{
	    if(vcpu_runnable(curinf->vcpu) )
		list_move_tail(LIST(curinf->vcpu), runq);
	    else
		list_move_tail(LIST(curinf->vcpu), inactiveq);
	}
	else
	    list_move(LIST(curinf->vcpu), runq);

    }

    loop_detection = 0;
/*
    list_for_each_safe ( cur, tmp, runq )
    {
	curinf = list_entry(cur,struct sc_vcpu_info,list);

	if(curinf->status & SC_SHIFT)
	{
	    curinf->period = curinf->period_new;
	    curinf->slice = curinf->slice_new;

	    dp_wrap_assign_pcpu(curinf->vcpu, ops);
	}
    }
*/

    // Activate it only when we are in Reverse Order. That way the
    // VM is started at the beginning of the queue of processor_a.
    list_for_each_safe ( cur, tmp, inactiveq )
    {
	if(loop_detection++ > 20)
	    printk("**** OOPS: Caught in an infinite loop: %d *****\n", __LINE__);

	curinf = list_entry(cur, struct sc_vcpu_info, list);
	curinf->status &= ~SC_INACTIVE;

	if(curinf->status & SC_ARRIVED)
	{
	    curinf->status &= ~SC_ARRIVED;
	    curinf->status |= SC_SPORADIC;
	}

	if(curinf->status & SC_SPORADIC && !(curinf->status & SC_SPLIT))
	    list_move_tail(LIST(curinf->vcpu), runq);
	else
	    list_move(LIST(curinf->vcpu), runq);

    }

    loop_detection = 0;


    prev = global_slice_start;

    if(!list_empty(runq))
	first = list_entry(runq->next, struct sc_vcpu_info, list);

    list_for_each_safe ( cur, tmp, runq )
    {
	if(loop_detection++ > 20)
	    printk("**** OOPS: Caught in an infinite loop: %d *****\n", __LINE__);

	curinf = list_entry(cur,struct sc_vcpu_info,list);

/*
	if(sc_debugging == 1)
	{
	    printk("------ CPU: %d - NOW: %ld - global_deadline: %ld - ID: %6d.%d - assigned cpu: %d - deadl: %ld - migrated: %d ------\n",
		    cpu,
		    now,
		    global_deadline,
		    curinf->vcpu->domain->domain_id,
		    curinf->vcpu->vcpu_id,
		    curinf->vcpu->processor,
		    now - get_local_deadl(curinf),
		    curinf->status & SC_MIGRATED);
	}
*/

	if(curinf->status & SC_RESET)
	{
	    si = (struct shared_info *) curinf->vcpu->domain->shared_info;

	    curinf->status &= ~SC_RESET;

	    // If extra_arg5 is zero, that means the guest didn't confirm the
	    // arrival, and so we reset the SC_ARRIVED flag, so that Xen
	    // can accept the next signal (vcpu_wake call) as the arrival.
	    // Ideally if this is done, we should also ignore the deadline
	    // value that we updated when we called vcpu_wake, and restore
	    // the original deadline value.

	    curinf->cputime = 0;
	}

	curinf->status &= ~SC_MIGRATED;

	/*
	if(curinf->local_cputime > 0 && first == curinf)
	{
	    curinf->local_cputime = (curinf->local_cputime > 500 ? 500 : curinf->local_cputime);
	    slice_length -= curinf->local_cputime;
	}
	else
	 */   curinf->local_cputime = 0;

	if(curinf->status & SC_SPLIT)
	{
	    // When we are in Reverse Order now, the VM will
	    // start at the beginning of processor_a's runqueue
	    // Otherwise, we force it to start at beginning
	    // of processor_b's runqueue.
	    // Essentially, the current reverse_order tells us
	    // in which RUNQ the VM ended up in before this
	    // function was called.
	    if(reverse_order_next < 0)
	    {
		if(curinf->processor_a != cpu)
		{
		    printk("* NR * %d.%d should be in cpu %d, but instead it's in cpu %d * * *\n",
			    curinf->vcpu->domain->domain_id,
			    curinf->vcpu->vcpu_id,
			    curinf->processor_a, cpu);
		}

		curr = (curinf->slice_a * slice_length);
		curr /= curinf->period_a;

		curinf->local_slice = curr + curinf->local_cputime;
		curinf->local_cputime = curinf->local_slice;
		prev = curinf->local_deadl = prev + curinf->local_cputime;

		curinf->local_slice -= 500;
		curinf->local_cputime = curinf->local_slice;

		curinf->status |= SC_MIGRATING;

		curr = (curinf->slice_b * slice_length);
		curr /= curinf->period_b;

		curinf->local_deadl_second = global_deadline;
		curinf->local_slice_second = curr;
	    }
	    else
	    {
		if(curinf->processor_b != cpu)
		{
		    printk("* R * %d.%d should be in cpu %d, but instead it's in cpu %d * * *\n",
			    curinf->vcpu->domain->domain_id,
			    curinf->vcpu->vcpu_id,
			    curinf->processor_b, cpu);
		}

		curr = (curinf->slice_b * slice_length);
		curr /= curinf->period_b;

		curinf->local_slice_second = curr + curinf->local_cputime;
		curinf->local_cputime = curinf->local_slice_second;
		prev = curinf->local_deadl_second = prev + curinf->local_cputime;

		curinf->local_slice_second -= 500;
		curinf->local_cputime = curinf->local_slice_second;

		curinf->status |= SC_MIGRATING;

		curr = (curinf->slice_a * slice_length);
		curr /= curinf->period_a;

		curinf->local_deadl = global_deadline;
		curinf->local_slice = curr;
	    }
	}
	else
	{
	    curr = (curinf->slice_new * slice_length);
	    curr /= curinf->period_new;

	    curinf->local_slice = curr + curinf->local_cputime;
	    curinf->local_cputime = curinf->local_slice;
	    prev = curinf->local_deadl = prev + curinf->local_cputime;

	    curinf->local_slice -= 500;
	    curinf->local_cputime = curinf->local_slice;
	}
/*
	if(curinf->local_deadl < now)
	    DPRINTK_ERR("[%d] *** BAD1 ***, assigning local_deadl that is before NOW - Diff: %ld ***\n", cpu, (now - curinf->local_deadl));

	if(curinf->status & SC_SPLIT)
	{
	    if(curinf->local_deadl_second < now)
		DPRINTK_ERR("[%d] *** BAD2 ***, assigning local_deadl_second that is before NOW - Diff: %ld ***\n", cpu, (now - curinf->local_deadl_second));
	}
*/
	if(cpu == 0)
	    DPRINTK2("- CPU: %d - NOW: %ld - gl. deadl.: %ld - ID: %6d.%d - lcl. deadl: %ld - slice: %lu -\n",
		    cpu,
		    now,
		    global_deadline,
		    curinf->vcpu->domain->domain_id,
		    curinf->vcpu->vcpu_id,
		    get_local_deadl(curinf),
		    curr);

	//if(!vcpu_runnable(curinf->vcpu) )
	//    list_move(LIST(curinf->vcpu), waitq);

    }

    loop_detection = 0;
/*
    if(sc_debugging == 1)
    {
	printk("-- END --- CPU: %d - %s - calculating CPU: %d - duration: %ld ------\n",
		smp_processor_id(),
		__func__,
		cpu,
		NOW() - new_now);
    }
    */
}

static void update_queues(int cpu, s_time_t now, const struct scheduler *ops)
{
    struct shared_info *si;
    struct sc_priv_info *prv = SC_PRIV(ops);
    //struct list_head     *migq     = MIGQ(cpu);
    struct list_head     *runq     = RUNQ(cpu);
    struct list_head     *waitq     = WAITQ(cpu);
    struct list_head     *cur, *tmp;
    struct sc_vcpu_info *inf;
    int migrate_to_processor;
    spinlock_t *lock;
    int loop_detection = 0;

    //DPRINTK3("------ Line: %d - CPU: %d - %s ------\n", __LINE__, smp_processor_id(), __func__);

    if(prv->status & SC_CPU0_BUSY)
	return;


    list_for_each_safe ( cur, tmp, runq )
    {
	if(prv->status & SC_CPU0_BUSY)
	   break;

	if(loop_detection++ > 25)
	{
	    printk("**** OOPS: Caught in an infinite loop: %d *****\n", __LINE__);
	    break;
	}

	inf = list_entry(cur,struct sc_vcpu_info,list);

	/*
	printk("-------- ARR - Got from %d.%d --- Used Time: %ld - Diff: %ld - Now: %ld --\n",
		inf->vcpu->domain->domain_id,
		inf->vcpu->vcpu_id,
		inf->cputime,
		inf->slice - inf->cputime,
		now);
*/
	if(inf->status & SC_SPORADIC && cpu == inf->vcpu->processor)
	{
	    if(now >= (CPU_INFO(cpu)->new_gl_d) || !sc_active(inf, 0) || !vcpu_runnable(inf->vcpu))
	    {
		if(inf->local_slice == 0)
		    inf->local_slice = inf->slice;

		list_move(LIST(inf->vcpu), waitq);
	    }
	    else if(inf->status & SC_SPLIT && inf->status & SC_MIGRATING && (inf->local_cputime - 500) < 0)
	    {
		// TODO: Needs locking to access/modify runqueues of other PCPUs
		if(inf->vcpu->processor == inf->processor_a)
		    migrate_to_processor = inf->processor_b;
		else
		    migrate_to_processor = inf->processor_a;

		DPRINTK3("-- From CPU: %d to %d - ID: %d.%d ----\n",
			smp_processor_id(),
			migrate_to_processor,
			inf->vcpu->domain->domain_id,
			inf->vcpu->vcpu_id);

		//DPRINTK2("--- get_local: %ld -- Now: %ld --\n", get_local_deadl(inf), now);

		lock = NULL;

		//DPRINTK2("--- smp: %d - inf->vcpu->processor: %d - migrate_to_processor: %d ---\n",
		//	smp_processor_id(),
		//	inf->vcpu->processor,
		//	migrate_to_processor);

		inf->status &= ~SC_MIGRATING;
		//inf->status &= ~SC_RUNNING;

		if(migrate_to_processor != inf->vcpu->processor && migrate_to_processor != smp_processor_id())
		{
		    //while(!lock)
		    //   lock = pcpu_schedule_trylock(migrate_to_processor);

		    inf->vcpu->processor = migrate_to_processor;
		    inf->status |= SC_MIGRATED;
		    list_move_tail(LIST(inf->vcpu), MIGQ(inf->vcpu->processor));

		    //pcpu_schedule_unlock(lock, inf->vcpu->processor);

		    // I"m not sure if this is enough. Maybe the sched_move_irqs happens only until
		    // the VM is activated,  which may be a little late.
		    if(CPU_INFO(inf->vcpu->processor)->new_gl_d == 0 || CPU_INFO(inf->vcpu->processor)->current_slice_expires == 0 ||
			    is_idle_vcpu(per_cpu(schedule_data, inf->vcpu->processor).curr) ||
			    (EDOM_INFO(per_cpu(schedule_data, inf->vcpu->processor).curr)->local_cputime < 0))
			cpu_raise_softirq(inf->vcpu->processor, SCHEDULE_SOFTIRQ);
		}
		else
		    printk("--- NOPE --- migrating to the same CPU --- \n");

		//DPRINTK4("--- After migrating ---\n");
	    }

	    // list_move(LIST(inf->vcpu), waitq);
	    //else if( !sc_active(inf, 0) || !vcpu_runnable(inf->vcpu) || inf->local_cputime < 0)
	    //else if(inf->local_cputime < 0)
	    //{
	    //	list_move_tail(LIST(inf->vcpu), runq);
	    //  }

	    si = (struct shared_info *) inf->vcpu->domain->shared_info;

	    if(si->extra_arg1[inf->vcpu->vcpu_id] > 0)
	    {
		if(si->extra_arg1[inf->vcpu->vcpu_id] == 1)
		{
		    if(inf->status & SC_SPORADIC)
		    {
			//if(!(inf->status & SC_ARRIVED))
			//{
			//    inf->status |= SC_ARRIVED;

			/*
			printk("-------- ARR - Got from %d.%d --- Used Time: %ld - Diff: %ld - Now: %ld --\n",
				inf->vcpu->domain->domain_id,
				inf->vcpu->vcpu_id,
				inf->cputime,
				inf->slice - inf->cputime,
				now);
				*/

			//curr = (inf->slice_new * slice_length);
			//curr /= inf->period_new;

			//inf->local_slice = curr;

			//}
		    }
		}
	    }

	    //if(inf->status & SC_ARRIVED)
	    //	list_move(LIST(inf->vcpu), runq);

	}
	else if((inf->local_cputime <= 0 || get_local_deadl(inf) <= now || now >= CPU_INFO(cpu)->new_gl_d) && cpu == inf->vcpu->processor)
	{
	    //FIXME: Hacky fix. I'm initializing local_slice to an initial value
	    // of cputime in order to force an entry to the if statement below,
	    // therefore forcing an entry in the calculate new global deadline function.
	    // This is just needed at the beginning when a new VCPU is added to a RUNQ
	    if(inf->local_slice == 0)
		inf->local_slice = inf->slice;

	    //          inf->cputime += get_local_slice(inf);

	    //if(inf->cputime >= inf->slice)
	    //      inf->cputime -= inf->slice;

	    if(inf->status & SC_SPLIT && inf->status & SC_MIGRATING)
	    {
		/* This is done because the rest of the stuff will be done when sc_context_saved is called */

		/*
		   if(EDOM_INFO(current) == inf)
		   {
		   list_move(LIST(inf->vcpu), waitq);
		   continue;
		   }
		   */

		// FIXME: It seems that global_deadline is being updated too quickly. So
		// quick that when a CPU other than zero, checks this condition, global_deadline
		// might have been advanced to the next global_deadline. Therefore,
		// a simple "fix" for this is to check for both the new or old global_deadline.
		//              if(get_local_deadl(inf) == global_deadline || get_local_deadl(inf) == global_slice_start)

		// TODO: Needs locking to access/modify runqueues of other PCPUs
		if(inf->vcpu->processor == inf->processor_a)
		    migrate_to_processor = inf->processor_b;
		else
		    migrate_to_processor = inf->processor_a;

		DPRINTK3("-- From CPU: %d to %d -  ID: %d.%d ---\n",
			smp_processor_id(),
			migrate_to_processor,
			inf->vcpu->domain->domain_id,
			inf->vcpu->vcpu_id);

		//DPRINTK2("--- get_local: %ld -- Now: %ld --\n", get_local_deadl(inf), now);

		lock = NULL;

		//DPRINTK2("--- smp: %d - inf->vcpu->processor: %d - migrate_to_processor: %d ---\n",
		//	smp_processor_id(),
		//	inf->vcpu->processor,
		//	migrate_to_processor);

		inf->status &= ~SC_MIGRATING;
		//inf->status &= ~SC_RUNNING;

		if(migrate_to_processor != inf->vcpu->processor && migrate_to_processor != smp_processor_id())
		{
		    //while(!lock)
		    //   lock = pcpu_schedule_trylock(migrate_to_processor);

		    inf->vcpu->processor = migrate_to_processor;
		    inf->status |= SC_MIGRATED;
		    list_move_tail(LIST(inf->vcpu), MIGQ(inf->vcpu->processor));

		    //pcpu_schedule_unlock(lock, inf->vcpu->processor);

		    // I"m not sure if this is enough. Maybe the sched_move_irqs happens only until
		    // the VM is activated,  which may be a little late.
		   // if(CPU_INFO(inf->vcpu->processor)->new_gl_d == 0 || CPU_INFO(inf->vcpu->processor)->current_slice_expires == 0)
		//	cpu_raise_softirq(inf->vcpu->processor, SCHEDULE_SOFTIRQ);

		    if(CPU_INFO(inf->vcpu->processor)->new_gl_d == 0 || CPU_INFO(inf->vcpu->processor)->current_slice_expires == 0 ||
				is_idle_vcpu(per_cpu(schedule_data, inf->vcpu->processor).curr) )
			    cpu_raise_softirq(inf->vcpu->processor, SCHEDULE_SOFTIRQ);

		}
		else
		    printk("--- NOPE --- migrating to the same CPU --- \n");

		//DPRINTK3("--- After migrating ---\n");
	    }
	    else
		list_move(LIST(inf->vcpu), waitq);

	    // list_move(LIST(inf->vcpu), waitq);
	}
    }

/*
    list_for_each_safe ( cur, tmp, migq )
    {
	inf = list_entry(cur,struct sc_vcpu_info,list);
	if(get_local_deadl(inf) <= now)
	{
	    if(inf->local_slice == 0)
		inf->local_slice = inf->slice;

	    inf->status &= ~SC_MIGRATED;
	    list_move(LIST(inf->vcpu), waitq);
	}
    }
    */
}

/*
static void update_current(int cpu, s_time_t now, const struct scheduler *ops)
{
    struct list_head     *waitq     = WAITQ(cpu);
    struct list_head     *runq     = RUNQ(cpu);
    int migrate_to_processor;
    spinlock_t *lock;
    struct list_head     *cur, *tmp;
    struct sc_vcpu_info *inf;

    list_for_each_safe ( cur, tmp, runq )
    {
	inf = list_entry(cur,struct sc_vcpu_info,list);

	if(get_local_deadl(inf) <= now)
	{
	    if(inf->local_slice == 0)
		inf->local_slice = inf->slice;

	    if(inf->status & SC_SPLIT && inf->status & SC_MIGRATING)
	    {


		if(inf->vcpu->processor == inf->processor_a)
		    migrate_to_processor = inf->processor_b;
		else
		    migrate_to_processor = inf->processor_a;

		DPRINTK2("-- CPU: %d - Switching to %d queue ---\n",
			smp_processor_id(),
			migrate_to_processor);

		DPRINTK2("--- get_local: %ld -- Now: %ld --\n", get_local_deadl(inf), now);

		lock = NULL;

		DPRINTK2("--- smp: %d - inf->vcpu->processor: %d - migrate_to_processor: %d ---\n",
			smp_processor_id(),
			inf->vcpu->processor,
			migrate_to_processor);

		inf->status &= ~SC_MIGRATING;
		inf->status &= ~SC_RUNNING;

		if(migrate_to_processor != inf->vcpu->processor && migrate_to_processor != smp_processor_id())
		{
		    while(!lock)
			lock = pcpu_schedule_trylock(migrate_to_processor);

		    inf->vcpu->processor = migrate_to_processor;
		    inf->status |= SC_MIGRATED;
		    list_move_tail(LIST(inf->vcpu), RUNQ(inf->vcpu->processor));

		    pcpu_schedule_unlock(lock, inf->vcpu->processor);

		    // I"m not sure if this is enough. Maybe the sched_move_irqs happens only until
		    // the VM is activated,  which may be a little late.
		    cpu_raise_softirq(inf->vcpu->processor, SCHEDULE_SOFTIRQ);
		}
		else
		    printk("--- NOPE --- migrating to the same CPU --- \n");

		DPRINTK2("--- After migrating ---\n");
	    }
	    else
		list_move(LIST(inf->vcpu), waitq);

	}

	return;
    }
}
*/

static void global_deadline_barrier(struct sc_barrier_t* b, int cpu_id, s_time_t now, const struct scheduler *ops)
{
    struct sc_vcpu_info *runinf, *runinf2, *curinf, *previnf;
    struct list_head     *cur, *tmp;
    s_time_t  new_global_start_value, new_global_deadline;
    s_time_t  l_cputime;
    s_time_t  l_sched_start_abs;
    unsigned long flags;
    struct shared_info *si;
    //u64 start, end;
    //int cpu_count, i;
    int i;
    struct sc_priv_info *prv = SC_PRIV(ops);
    unsigned int nr_cpus = cpumask_last(&cpu_online_map) + 1;

    DPRINTK4("------ CPU: %d - %s - %d ------\n",
	    cpu_id,
	    __func__,
	    __LINE__);

    previnf = NULL;

    // Returns true if updating_global_deadline is changed to zero.


    //if(sc_debugging == 1 && smp_processor_id() < 2)
//	printk("-- CPU: %d - DEBUG1 Time: %ld ---\n", smp_processor_id(), NOW());

    new_global_start_value = global_deadline;
    new_global_deadline = global_deadline;

    if(cpu_id == 0)
    {
	// If the one you got is not the current global_deadline
	// then it was already updated. Go ahead and do calculations.

	if(CPU_INFO(cpu_id)->new_gl_d != global_deadline)
	{
	    goto do_calculations;
	}

	spin_lock_irqsave(&prv->lock, flags);
	prv->status |= SC_CPU0_BUSY;

	if( !list_empty(&deadline_queue) )
	{
check_runinf_again:
	    runinf = list_entry(deadline_queue.next, struct sc_vcpu_info, d_list);
	    //runinf = MIN_HEAP[0].data;

	    si = (struct shared_info *) runinf->vcpu->domain->shared_info;

/*	    if(runinf->status & SC_RUNNING)
	    {
		//FIXME: I'm not sure if we really need for runinf to stop running. The reason why I made
		//sure it wasn't running was because I print a value for debugging purposes inside this
		//function. And if the runinf has not stopped running I will print wrong value.
		printk("-- Fix this if you want the debugging numbers to be correct --\n");
		atomic_dec(&b->updating_global_deadline);
		return;
	    }
*/
	    l_sched_start_abs = runinf->sched_start_abs;

	    if(runinf->status & SC_RUNNING)
		l_cputime = runinf->slice - (runinf->cputime + (now - l_sched_start_abs));
	    else
		l_cputime = runinf->cputime;

	    /*
	    if(runinf->vcpu->domain->domain_id != 0 && sc_debugging == 1 && !(runinf->status & SC_UPDATE_DEADL))
	    {
		printk("-------- DBG - Got from %d.%d --- Used Time: %ld - Diff: %ld ---\n",
			runinf->vcpu->domain->domain_id,
			runinf->vcpu->vcpu_id,
			l_cputime,
			runinf->slice - l_cputime);
	    }
	    */

	    /*
	    if(runinf->vcpu->domain->domain_id != 0 && sc_debugging == 1)
	    {
		if(runinf->status & SC_RUNNING)
		{
		    printk("-R- Iter: %ld - Deadline: %ld  expired for VCPU: %d.%d - dead_diff: %ld - slice_left: %ld ---\n",
			    si->extra_arg2[runinf->vcpu->vcpu_id],
			    runinf->deadl_abs,
			    runinf->vcpu->domain->domain_id,
			    runinf->vcpu->vcpu_id,
			    now - runinf->deadl_abs,
			    runinf->slice - (l_cputime + (now - l_sched_start_abs)));
		}
		else
		{
		    printk("--- Iter: %ld - Deadline: %ld  expired for VCPU: %d.%d - dead_diff: %ld - slice_left: %ld ---\n",
			    si->extra_arg2[runinf->vcpu->vcpu_id],
			    runinf->deadl_abs,
			    runinf->vcpu->domain->domain_id,
			    runinf->vcpu->vcpu_id,
			    now - runinf->deadl_abs,
			    runinf->slice - runinf->cputime);
		}
	    }

	    si->extra_arg2[runinf->vcpu->vcpu_id]++;
*/

//	    runinf->cputime = 0;
	    // TODO: Figure out the effect of leaving deadl_abs as extra_arg2 or incrementing it by
	    // runinf->period like we do here.

	    if(!(runinf->status & SC_UPDATE_DEADL))
		runinf->status |= SC_RESET;

	    //FIXME: We should the adjust hypercall to change a VCPU from sporadic to
	    // non-sporadic, instead of waiting for the global deadline to be reached
	    // JC_BUG: When a task arrives, we need to notify immediately or we need to check
	    // all the VCPUs in the sc_list, for anyone with a sporadic task running. Since a sporadic
	    // task can start runnint at any time, then we won't be able to know it arrived until
	    // we get to the old deadline of the VCPU.
	    // IDEA: I don't think the above should cause any deadline misses. But I'll verify.
	    if(si->extra_arg1[runinf->vcpu->vcpu_id] > 0)
	    {
		/*
		if(si->extra_arg1[runinf->vcpu->vcpu_id] == 1)
		{
		    if(!(runinf->status & SC_SPORADIC))
		    {
			//runinf->status  |= SC_SPORADIC;
			runinf->status  |= SC_ARRIVED;
			//JC: FIXME This is not efficient!!!
			list_move_tail(&runinf->sc_list, &sc_list_head);
			prv->status |= SC_SHIFT;
		    }
		}
		else if(runinf->status & SC_SPORADIC)
		{
		    runinf->status &= ~SC_SPORADIC;
		    //runinf->status &= ~SC_ARRIVED;

		    list_move(&runinf->sc_list, &sc_list_head);
		    prv->status |= SC_SHIFT;
		}
		*/

		runinf->status &= ~SC_UPDATE_DEADL;

		//printk("-------- DomU deadl: %lu - RTA deadl: %lu ---------\n", runinf->deadl_abs, si->extra_arg3[runinf->vcpu->vcpu_id]);
		runinf->deadl_abs = si->extra_arg3[runinf->vcpu->vcpu_id];
		si->extra_arg4[runinf->vcpu->vcpu_id] = runinf->deadl_abs;
		si->extra_arg3[runinf->vcpu->vcpu_id] = 0;

		si->extra_arg1[runinf->vcpu->vcpu_id] = 0;
		si->extra_arg2[runinf->vcpu->vcpu_id] = 0;
	    }
	    else if(runinf->status & SC_UPDATE_DEADL)
	    {
		runinf->status &= ~SC_UPDATE_DEADL;
	    }
	    else
	    {
		// If the VCPU is sporadic, and it is in arrival mode
		// then we can't just update its deadl_abs value as usual
		// e.g. 	runinf->deadl_abs += runinf->period;
		// instead we need to leave it as is, and reinsert it into
		// the queue so that it is resorted.

		runinf->deadl_abs += runinf->period;
		si->extra_arg4[runinf->vcpu->vcpu_id] = runinf->deadl_abs;

		if(si->extra_arg5[runinf->vcpu->vcpu_id] > 0) // Is there an RTA running?
		{
		    if(si->extra_arg3[runinf->vcpu->vcpu_id])
		    {
			/*
			 * printk("-------- DBG - DomU deadl: %lu - RTA deadl: %lu  - Next deadl: %lu - Now: %lu ---------\n",
			 runinf->deadl_abs,
			 si->extra_arg3,
			 si->extra_arg5,
			 now);
			 printk("-------- DBG - Got from DomU %d.%d - %ld ---\n",
			 runinf->vcpu->domain->domain_id,
			 runinf->vcpu->vcpu_id,
			 si->extra_arg3[runinf->vcpu->vcpu_id] - now);
			 */

			runinf->deadl_abs = si->extra_arg3[runinf->vcpu->vcpu_id];
			si->extra_arg4[runinf->vcpu->vcpu_id] = runinf->deadl_abs;
			si->extra_arg3[runinf->vcpu->vcpu_id] = 0;
		    }
		    else
		    {
			//if(previnf != runinf)
			//    printk("-------- DBG - Got nothing from DomU %d.%d  ---\n",
			//	    runinf->vcpu->domain->domain_id,
			//	    runinf->vcpu->vcpu_id);
			si->extra_arg3[runinf->vcpu->vcpu_id] = 0;
		    }
		}
		//else
		//{
		//    runinf->status &= ~SC_ARRIVED;
		//}
	    }


	    //FIXME: Shouldn't do this loop, unless there's a bug
	    while(runinf->deadl_abs <= now)
	    {
		printk("--- Skip - %d.%d - %ld ---\n",
			runinf->vcpu->domain->domain_id,
			runinf->vcpu->vcpu_id,
			now - runinf->deadl_abs);
		//BUG_ON(1);

		if(runinf->deadl_abs == 0)
		    runinf->deadl_abs = now;
		else
		    runinf->deadl_abs += runinf->period;

		si->extra_arg4[runinf->vcpu->vcpu_id] = runinf->deadl_abs;
		si->extra_arg3[runinf->vcpu->vcpu_id] = 0;
	    }

	    if(task_on_deadline_queue(runinf->vcpu))
		list_del_init(D_LIST(runinf->vcpu));

	    list_insert_sort(&deadline_queue, D_LIST(runinf->vcpu), runq_comp);
	    runinf   = list_entry(deadline_queue.next, struct sc_vcpu_info, d_list);

	    //updateMin(runinf);
	    //runinf   = MIN_HEAP[0].data;

	    if(runinf->status & SC_UPDATE_DEADL)
	    {
		if(task_on_deadline_queue(runinf->vcpu))
		    list_del_init(D_LIST(runinf->vcpu));

		runinf->status &= ~SC_UPDATE_DEADL;

		list_insert_sort(&deadline_queue, D_LIST(runinf->vcpu), runq_comp);
		runinf   = list_entry(deadline_queue.next, struct sc_vcpu_info, d_list);

		//updateMin(runinf);
		//runinf   = MIN_HEAP[0].data;
	    }

	    previnf = runinf;

	    /*
	       si = (struct shared_info *) runinf->vcpu->domain->shared_info;

	       if(si->extra_arg4)
	       {
	       printk("-------- DBG - DomU deadl: %lu - RTA deadl: %lu  - Next deadl: %lu - Now: %lu ---------\n",
	       runinf->deadl_abs,
	       si->extra_arg4,
	       si->extra_arg5,
	       now);
	       si->extra_arg4 = 0;
	       }
	    */

	    global_slice_start = new_global_start_value;

	    if( (runinf->deadl_abs - now) < 250000)
	    {
		//DPRINTK("*** BAD3 ***: Global slice might be too small: %ld ***\n", runinf->deadl_abs - global_slice_start);

		runinf2  = list_entry(deadline_queue.next->next, struct sc_vcpu_info, d_list);
		//runinf2  = MIN_HEAP[0].data;

		if((runinf2->deadl_abs - now) < 250000)
		    goto check_runinf_again;
		else
		    new_global_deadline = now + 250000;
	    }
	    else
		new_global_deadline = runinf->deadl_abs;

	    //reverse_order_next = reverse_order_next * -1;
	    reverse_order_next = 1;


/*
	       printk("-A.1- CPU: %d - now: %ld - old: %ld - new global_deadline: %ld - ID: %6d.%d - local_deadl: %ld - %s ------\n",
	       cpu_id,
	       now,
	       global_slice_start,
	       global_deadline,
	       runinf->vcpu->domain->domain_id,
	       runinf->vcpu->vcpu_id,
	       runinf->local_deadl,
	       __func__);
*/	}
	else
	{
	    printk("-- BAD -- A.2- Deadline queue is empty ---\n");
	    //BUG_ON(1);
//	    new_global_deadline = now;
	    global_slice_start = global_deadline;
	    new_global_deadline += 1000000;
	}

	while(new_global_deadline <= now) {
	    printk("-- BAD -- CPU: %d - Oops, global_deadline is very behind, by: %ld --\n", cpu_id, new_global_deadline - now);
	    //BUG_ON(1);
	    global_slice_start = global_deadline;
	    new_global_deadline += 1000000;
	}

	if(prv->status & SC_SHIFT)
	{
	    printk("-- Reseting CPUs BWs - DOM0_CPU_COUNT: %d - Online CPUS: %d - reverse: %d ---\n", dom0_cpu_count, nr_cpus, reverse_order_next);
	    for(i = dom0_cpu_count; i < nr_cpus; i++)
	    {
		HSLICE(i) = 0;
		HPERIOD(i) = 100000;
	    }
	}

	for(i = dom0_cpu_count; i < nr_cpus; i++)
	{
	    CPU_INFO(i)->used_slice = 0;
	    CPU_INFO(i)->used_period = 100000;
	}

	list_for_each_safe ( cur, tmp, &sc_list_head )
	{
	    curinf = list_entry(cur, struct sc_vcpu_info, sc_list);

	    if(prv->status & SC_SHIFT)
	    {
		curinf->period_new = curinf->period_temp;
		curinf->slice_new  = curinf->slice_temp;

		curinf->slice_new = (100000 * curinf->slice_new) / curinf->period_new;
		curinf->period_new = 100000;

		curinf->period = curinf->period_temp * 1000;
		curinf->slice = curinf->slice_temp * 1000;

		dp_wrap_assign_pcpu(curinf->vcpu, ops);
	    }
	    curinf->status &= ~SC_WOKEN;
	    set_cpu_bw_reservation(curinf->vcpu);
	}


	new_global_start_value = NOW();
	global_slice_start = new_global_start_value;

	global_deadline = new_global_deadline;
	prv->status &= ~SC_SHIFT;
	prv->status &= ~SC_CPU0_BUSY;
	spin_unlock_irqrestore(&prv->lock, flags);

	for(i = dom0_cpu_count; i <= last_assigned_pcpu; i++)
	{
	    //printk("--- JC: Calling cpu_raise_softirq ---\n");
//	    if(is_idle_vcpu(per_cpu(schedule_data, i).curr))
		cpu_raise_softirq(i, SCHEDULE_SOFTIRQ);
	}

	//printk("--- START CALC --- Global Slice %ld ---\n", global_deadline - global_slice_start);
	//atomic_dec(&b->updating_global_deadline);
    }
    else
    {
	//atomic_dec(&b->updating_global_deadline);
	//atomic_dec(&b->cpu_count);

	/* printk("------ THIS CPU: %d - PASSED CPU: %d - %s ------\n",
	    smp_processor_id(),
	    cpu_id,
	    __func__);
*/
	//while(atomic_read(&b->updating_global_deadline) != -1);
/*
	printk("------ THIS CPU: %d - PASSED CPU: %d - %s ------\n",
	    smp_processor_id(),
	    cpu_id,
	    __func__);
*/
	if(CPU_INFO(cpu_id)->new_gl_d == global_deadline)
	    return;
    }

    //if(sc_debugging == 1 && smp_processor_id() < 2)
	//printk("-- CPU: %d - DEBUG2 Time: %ld ---\n", smp_processor_id(), NOW());

do_calculations:

    //if(cpu_id != 0)
	calculate_new_local_deadlines(cpu_id, now, ops);
    //update_queues(cpu_id, now, ops);
    CPU_INFO(cpu_id)->new_gl_d = global_deadline;
}

static struct task_slice sc_do_schedule(
	const struct scheduler *ops, s_time_t now, bool_t tasklet_work_scheduled)
{
    int array_index, i, cpu_i;
    //int array_index, cpu_i;
    int                   cpu      = smp_processor_id();
    struct list_head     *runq     = RUNQ(cpu);
    //struct list_head     *waitq     = WAITQ(cpu);
    struct list_head     *migq     = MIGQ(cpu);
    struct sc_vcpu_info *inf     = EDOM_INFO(current);
    struct sc_vcpu_info *runinf;
    struct task_slice      ret;
    s_time_t              new_now;
    //struct shared_info *si;
    struct sc_priv_info *prv = SC_PRIV(ops);
    //unsigned long flags;


    /*
     * Create local state of the status of the domain, in order to avoid
     * inconsistent state during scheduling decisions, because data for
     * vcpu_runnable is not protected by the scheduling lock!
*/
    DPRINTK4("------ THIS CPU: %d - START -----\n",
	    cpu);

    //update_current(cpu,now, ops);

///    if(cpu != 0)
	//spin_lock_irqsave(&prv->lock, flags);

    CPU_INFO(cpu)->allocated_time += now - inf->sched_start_abs;

    if ( !list_empty(migq) )
    {
	runinf   = list_entry(migq->next,struct sc_vcpu_info,list);

	//if(CPU_INFO(cpu)->new_gl_d > now  && !(prv->status & SC_CPU0_BUSY) && cpu == runinf->vcpu->processor)
	if(!(prv->status & SC_CPU0_BUSY) && cpu == runinf->vcpu->processor)
	{
	    runinf->local_cputime = get_local_slice(runinf);
	    //runinf->local_cputime = (CPU_INFO(cpu)->new_gl_d - now);
	    //list_move(LIST(runinf->vcpu), runq);
	    if(runinf->status & SC_MIGRATING)
		list_move(LIST(runinf->vcpu), runq);
	    else
		list_move_tail(LIST(runinf->vcpu), runq);

	}
    }

    if( !is_idle_vcpu(current) && !(prv->status & SC_CPU0_BUSY) && inf->vcpu->processor == cpu)
    {
	inf->local_cputime -= now - inf->sched_start_abs;
	inf->cputime += now - inf->sched_start_abs;
	inf->status |= SC_ASLEEP;

	//FIXME: This should only be done for SPORADIC VMS

	if(inf->status & SC_SPORADIC)
	{
	    if(inf->local_cputime < 0)
		list_move_tail(LIST(inf->vcpu), runq);
	}
    }

    //if(inf->status & SC_SPORADIC)
    //	sporadic_update_queues(cpu,now, ops);
    //   else
    update_queues(cpu,now, ops);

    //if(cpu != 0)
//	spin_unlock_irqrestore(&prv->lock, flags);


    //if ( inf->status & SC_ASLEEP )
    //	inf->block_abs = now;

//    inf->local_cputime -= now - inf->sched_start_abs;

    //inf->status &= ~SC_RUNNING;

//choose_next_task:

    if(cpu == 0)
    {
	if(prv->status & SC_SHIFT)
	{
	    if(CPU_INFO(cpu)->new_gl_d + 15000 <= now)
		global_deadline_barrier(&prv->cpu_barrier, cpu, now, ops);
	}
	else
	{
	    if(CPU_INFO(cpu)->new_gl_d  == 0)
		global_deadline_barrier(&prv->cpu_barrier, cpu, now, ops);
	    else if(CPU_INFO(cpu)->new_gl_d <= now)
		global_deadline_barrier(&prv->cpu_barrier, cpu, now, ops);
	}
    }
    else
    {
	if(CPU_INFO(cpu)->new_gl_d <= now)
	    global_deadline_barrier(&prv->cpu_barrier, cpu, now, ops);
    }

    new_now = NOW();
    if ( tasklet_work_scheduled ||
	    unlikely(!cpumask_test_cpu(cpu,
		    cpupool_scheduler_cpumask(per_cpu(cpupool, cpu)))) )
    {
	ret.task = IDLETASK(cpu);
	//ret.time = SECONDS(1);
	ret.time = EXTRA_QUANTUM;
    }
    //else if (!list_empty(runq))
    else if (!list_empty(runq) && (CPU_INFO(cpu)->new_gl_d >= (now+5000) || cpu == 0) && !(prv->status & SC_CPU0_BUSY))
    {
	runinf   = list_entry(runq->next,struct sc_vcpu_info,list);
	//last   = list_entry(sc_list_head.prev,struct sc_vcpu_info, sc_list);

//	if( ((runinf->vcpu->is_running && runinf == inf) || (!runinf->vcpu->is_running)) &&
//	(sc_active(runinf, now) && vcpu_runnable(runinf->vcpu)) )

	if(sc_active(runinf, now) && vcpu_runnable(runinf->vcpu) && (!(runinf->vcpu->is_running) || runinf == inf))
	{
	    ret.task = runinf->vcpu;

	    if(runinf->status & SC_SPORADIC)
	    {
		//if(runinf->status & SC_UPDATE_DEADL && !(runinf->status & SC_ARRIVED))
		//    ret.time = MILLISECS(1);
		//else
		ret.time = get_local_slice(runinf);

		ret.time = (runinf->local_cputime < ret.time ? runinf->local_cputime: ret.time);

		ret.time = (ret.time < 0 ? MILLISECS(10) : ret.time);

		ret.time = (ret.time + now <= (CPU_INFO(cpu)->new_gl_d) ?
			ret.time : ((CPU_INFO(cpu)->new_gl_d - now)) );
	    }
	    else
		ret.time = get_local_deadl(runinf) - now;

	    /*
	    if(runinf->status & SC_MIGRATED)
		ret.time = CPU_INFO(cpu)->new_gl_d - now;
	    else if(!(runinf->status & SC_SPLIT) && reverse_order_next > 0 && runinf == last)
		ret.time = CPU_INFO(cpu)->new_gl_d - now;
	    else
	*/	//ret.time = (runinf->local_cputime + now <= CPU_INFO(cpu)->new_gl_d ? runinf->local_cputime : CPU_INFO(cpu)->new_gl_d - now);

	    if(cpu == 0)
		ret.time = (global_deadline - now);
	}
	else
	{
	    if(runinf->status & SC_SPORADIC)
		ret.time = (MILLISECS(100) + now <= CPU_INFO(cpu)->new_gl_d ? MILLISECS(100) : CPU_INFO(cpu)->new_gl_d - now);
	    else
		ret.time = get_local_deadl(runinf) - now;

	    ret.task = IDLETASK(cpu);

	    if(runinf->vcpu->is_running)
	    {
		//printk("--- %d.%d is still running --\n", runinf->vcpu->domain->domain_id, runinf->vcpu->vcpu_id);
		ret.time = MICROSECS(4);
	    }
	}
    }
    else
    {
	ret.task = IDLETASK(cpu);

	if(CPU_INFO(cpu)->new_gl_d != 0)
	    ret.time = (MILLISECS(100) + now <= CPU_INFO(cpu)->new_gl_d ? MILLISECS(100) : CPU_INFO(cpu)->new_gl_d - now);
	else
	    ret.time = MICROSECS(4);

	//ret.time = MILLISECS(1);
	//ret.time = CPU_INFO(cpu)->new_gl_d - now;
    }

    /*
     * TODO: Do something USEFUL when this happens and find out, why it
     * still can happen!!!
     */
    if ( ret.time < 5000)
    {
	/*
	printk("--- CPU: %d - Ouch! We are seriously BEHIND schedule! %"PRIi64"\n",
		cpu, ret.time);
	printk("------ CPU: %d - NOW: %ld - ID: %6d.%d - Slice: %ld - g_deadl: %ld - %s ------\n",
	    cpu,
	    new_now - now,
	    ret.task->domain->domain_id,
	    ret.task->vcpu_id,
	    ret.time,
	    global_deadline,
	    __func__);
	    */
	//ret.time = (MICROSECS(5) + new_now <= CPU_INFO(cpu)->new_gl_d ? MICROSECS(5) : CPU_INFO(cpu)->new_gl_d - new_now);
	//if(CPU_INFO(cpu)->new_gl_d != 0)
	//   ret.time = (MICROSECS(5) + new_now <= CPU_INFO(cpu)->new_gl_d ? MICROSECS(5) : CPU_INFO(cpu)->new_gl_d - new_now);
	//else

	// FIXME: Note cpu 0 should never get here.
	    ret.time = MICROSECS(5);
    }

    if(EDOM_INFO(ret.task)->status & SC_MIGRATED)
    {
	EDOM_INFO(ret.task)->status &= ~SC_MIGRATED;
	//ret.migrated = 1;
	ret.migrated = 0;
//	printk("=== MIGRATED, BABY ===\n");
    }
    else
	ret.migrated = 0;

    if(sc_debugging == 1)
    {
	if(CPU_INFO(cpu)->d_array_index < DEBUG_LINES)
	{
	    array_index = CPU_INFO(cpu)->d_array_index;
	    array_index--;

	    if(ret.task->domain->domain_id != 32767 || (array_index > 0 && CPU_INFO(cpu)->d_array[array_index].domid != ret.task->domain->domain_id))
	    {
		array_index = CPU_INFO(cpu)->d_array_index;
		CPU_INFO(cpu)->d_array[array_index].domid = ret.task->domain->domain_id;
		CPU_INFO(cpu)->d_array[array_index].vcpuid = ret.task->vcpu_id;
		CPU_INFO(cpu)->d_array[array_index].now_time = new_now - now;
		CPU_INFO(cpu)->d_array[array_index].ret_time = ret.time;
		CPU_INFO(cpu)->d_array[array_index].slice_time = EDOM_INFO(ret.task)->local_cputime;
		CPU_INFO(cpu)->d_array[array_index].alloc = now - inf->sched_start_abs;
		CPU_INFO(cpu)->d_array_index++;

/*
		printk("-DEBUG: %d %ld %7d. %ld %ld %ld -\n",
			cpu,
			new_now - now,
			ret.task->domain->domain_id,
			ret.time,
			EDOM_INFO(ret.task)->local_cputime,
			now - inf->sched_start_abs);
 */
	    }
	}
	else
	{
	    sc_debugging = 3;
	}
    }
    else if(sc_debugging < 1 && cpu == 0)
    {
	if( (sc_debugging*-1) <= last_assigned_pcpu && (sc_debugging*-1) < 9)
	{
	    cpu_i = (sc_debugging*-1);

	    for(i = CPU_INFO(cpu_i)->print_index;
		    i < CPU_INFO(cpu_i)->print_index + 250 && i < DEBUG_LINES; i++)
	    {
		printk("- %d %ld %7d.%d %ld %ld %ld -\n",
			cpu_i,
			CPU_INFO(cpu_i)->d_array[i].now_time,
			CPU_INFO(cpu_i)->d_array[i].domid,
			CPU_INFO(cpu_i)->d_array[i].vcpuid,
			CPU_INFO(cpu_i)->d_array[i].ret_time,
			CPU_INFO(cpu_i)->d_array[i].slice_time,
			CPU_INFO(cpu_i)->d_array[i].alloc);

		if(CPU_INFO(cpu_i)->d_array[i].alloc == 0)
		{
		    i = -1;
		    break;
		}

		CPU_INFO(cpu_i)->d_array[i].now_time = 0;
		CPU_INFO(cpu_i)->d_array[i].domid = 0;
		CPU_INFO(cpu_i)->d_array[i].vcpuid = 0;
		CPU_INFO(cpu_i)->d_array[i].ret_time = 0;
		CPU_INFO(cpu_i)->d_array[i].slice_time = 0;
		CPU_INFO(cpu_i)->d_array[i].alloc = 0;
	    }

	    if(i == -1 || i == DEBUG_LINES)
	    {
		CPU_INFO(cpu_i)->d_array_index = 0;
		CPU_INFO(cpu_i)->print_index = 0;
		sc_debugging--;
	    }
	    else
		CPU_INFO(cpu_i)->print_index = i;
	}
	else
	    sc_debugging = 4;
    }

    EDOM_INFO(ret.task)->sched_start_abs = now;
    EDOM_INFO(ret.task)->status |= SC_RUNNING;
    CHECK(ret.time > 0);
    ASSERT(sc_runnable(ret.task));
    CPU_INFO(cpu)->current_slice_expires = now + ret.time;

    DPRINTK4("------ THIS CPU: %d - END -----\n",
	    cpu);

    return ret;
}


static void sc_sleep(const struct scheduler *ops, struct vcpu *d)
{
    struct list_head     *waitq     = WAITQ(d->processor);

    DPRINTK3("------ CPU: %d - ID: %6d.%d - %s ------\n",
	    smp_processor_id(),
	    d->domain->domain_id,
	    d->vcpu_id,
	    __func__);

    if ( is_idle_vcpu(d) )
	return;

    EDOM_INFO(d)->status |= SC_ASLEEP;

    if(EDOM_INFO(d)->status & SC_SPORADIC)
	list_move_tail(LIST(d), waitq);

    if(unlikely(!__task_on_sclist(d)) && d->domain->domain_id != 0)
	list_add_tail(&(EDOM_INFO(d)->sc_list), &sc_list_head);

    if ( per_cpu(schedule_data, d->processor).curr == d )
    {
	cpu_raise_softirq(d->processor, SCHEDULE_SOFTIRQ);
    }
}

#define DOMAIN_EDF   1
#define DOMAIN_IDLE   4
static inline int get_run_type(struct vcpu* d)
{
    if (is_idle_vcpu(d))
	return DOMAIN_IDLE;
    return DOMAIN_EDF;
}

static void sc_wake(const struct scheduler *ops, struct vcpu *d)
{
    struct shared_info *si;
    struct sc_priv_info *prv = SC_PRIV(ops);
    unsigned long flags;
    s_time_t              now = NOW();
    s_time_t slice_length;
    s_time_t curr;
    struct sc_vcpu_info* inf = EDOM_INFO(d);

    DPRINTK3("------ CPU: %d - ID: %6d.%d - %s - time: %ld -----\n",
	    smp_processor_id(),
	    d->domain->domain_id,
	    d->vcpu_id,
	    __func__,
	    now);

    if ( unlikely(is_idle_vcpu(d)) )
	return;

    spin_lock_irqsave(&prv->lock, flags);

    if(prv->status & SC_CPU0_BUSY)
    {
	printk("--- DEBUGGING: calling sc_wake while CPU 0 is activated ---\n");

	// JC HACK: FIXME: This is a rudimentary spinlock. Should be better implemented. We
	// use a while instead of a return, to wait for CPU 0 to finish moving stuff around.
	// If during this time it detects that the woken VCPU is runnable it will do what needs
	// to be done (e.g. move it the corresponding runq, or mark it as split). If it did not
	// detect that is was runnable we will take care of moving.
	// TODO: Moving a split VCPU may happen a i little too late, and we might miss a deadline

	//if(smp_processor_id() != 0)
	   //while(prv->status & SC_CPU0_BUSY);

	//return;
    }

//    spin_unlock_irqrestore(&prv->lock, flags);

//    now = NOW();
    //slice_length = CPU_INFO(d->processor)->new_gl_d - now;
    slice_length = global_deadline - now;

    ASSERT(!sc_runnable(d));
    inf->status &= ~SC_ASLEEP;

    if ( unlikely(inf->deadl_abs == 0) )
    {
	/* Initial setup of the deadline */
	inf->deadl_abs = now + inf->period;

	/*
	inf->deadl_abs = now + 200000000;
	inf->deadl_abs = inf->deadl_abs - (inf->deadl_abs % 100000000);
	*/

	//spin_lock_irqsave(&prv->lock, flags);

	// Done one time.
	if(global_deadline == 0)
	    global_deadline = now;

	list_insert_sort(&deadline_queue, D_LIST(d), runq_comp);
	//heapInsert(inf);
	//spin_unlock_irqrestore(&prv->lock, flags);

	if(!__task_on_queue(d))
	{
	    printk("What should we do here?---\n");
	    list_add_tail(LIST(d), INACTIVEQ(d->processor));
	}

	if(unlikely(!__task_on_sclist(d)) && d->domain->domain_id != 0)
	    list_add_tail(&inf->sc_list, &sc_list_head);
    }
    else
    {
	if(inf->status & SC_SPORADIC)
	{
	    si = (struct shared_info *) inf->vcpu->domain->shared_info;

	    if(inf->status & SC_UPDATE_DEADL)// || inf->status & SC_ARRIVED)
	    {
		// FIXME: I think we get here if the VM woke up and we recalculated its
		// deadline and local_cputime value, but it went to sleep.

		if(inf->status & SC_SPLIT)
		{
		    curr = (inf->slice_b * slice_length);
		    curr /= inf->period_b;

		    inf->local_slice_second = curr;
		    inf->local_cputime = inf->local_slice_second;

		    if(!(inf->status & SC_MIGRATED))
			inf->status |= SC_MIGRATING;

		    curr = (inf->slice_a * slice_length);
		    curr /= inf->period_a;

		    inf->local_deadl = global_deadline;
		    inf->local_slice = curr;
		}
		else
		{
		    curr = (inf->slice_new * slice_length);
		    curr /= inf->period_new;

		    inf->local_slice = curr;
		    inf->local_cputime = inf->local_slice;

		    // TODO: Verify the effect of this. The idea is that if we are
		    // idle, then we shouldn't worry whether this VCPU will use more
		    // bw than it should. If the CPU is idle we should let it use idle PCPU
		    // BW.
		    /*
		    // We choose the minimum of BW*time_left or local_cputime
		    if(inf->local_slice < inf->local_cputime)
			inf->local_cputime = inf->local_slice;
		    else
			inf->local_slice = inf->local_cputime;
			*/
		}
	    }
	    else
	    {
		// Update the deadline of the VM
	//	inf->deadl_abs = now + inf->period;
		if(!(inf->status & SC_WOKEN))
		{
		    dynamic_reservation(inf->vcpu);

		    inf->status |= SC_WOKEN;

		    if(inf->status & SC_SPLIT)
		    {
			curr = (inf->slice_b * slice_length);
			curr /= inf->period_b;

			inf->local_slice_second = curr;
			inf->local_cputime = inf->local_slice_second;

			inf->status |= SC_MIGRATING;

			curr = (inf->slice_a * slice_length);
			curr /= inf->period_a;

			inf->local_deadl = global_deadline;
			inf->local_slice = curr;
		    }
		    else
		    {
			// TODO: Recalculate the local subslice
			curr = (inf->slice_new * slice_length);
			curr /= inf->period_new;

			inf->local_slice = curr;
			inf->local_cputime = inf->local_slice;

			// EXPLANATION: This is a hack which essentially uses idle time and gives it to the
			// VCPUS if they are runnable. Really, the main problem in my tests is that there's
			// other tasks using up the time guaranteed to the VCPU. These tasks (ssh, timeout, etc)
			// steal time from the RTA. That plus the fact that the RTA's arrival task is later
			// than the arrival time registered by sc_wake (it uses the external event), means
			// that the time given to the VCPU is used earlier and not by the RTA. The time given
			// to the VCPU is correct, and it would work if the RTA arrived at the time used by
			// sc_wake. How do we fix this?
			//JC_BUG();

			////IDEA: Let's leave local_cputime to be the original guaranteed value given
			//to the VCPU as if it were periodic. This means that only the sporadic VCPUS
			//which are done using local_cputime will be moved to waitq. However, the VCPU will
			//only be scheduled the value calculated in local_slice (in sc_wake) or the value
			//in local_cputime (whichever is minimum). After this time, schedule will be called
			//again, and the VCPU will be put at the end of the list, and other VCPUs which are
			//runnable should be given priority..
			//JC_BUG();
			//inf->local_cputime = inf->local_slice;
		    }
		    // TODO: Setup a flag to mark this VM as arrived:
		    //  1. During the arrival mode, we can't run this code again, until the VM meets this new deadline
		    //  2. Or we can rerun this code before the new deadline only if the guest VM tells us.

		    //inf->status |= SC_UPDATE_DEADL;
		    //inf->cputime = 0;
		    //
		}
	    }

	    inf->status &= ~SC_RUNNING;

	    if(inf->status & SC_MIGRATING)
		list_move(LIST(inf->vcpu), MIGQ(d->processor));
	    else
		list_move(LIST(inf->vcpu), RUNQ(d->processor));

	    //if(inf->local_cputime > 5000)
	}
//	else if(get_local_deadl(inf) > now && now < CPU_INFO(d->processor)->new_gl_d)
    }

    spin_unlock_irqrestore(&prv->lock, flags);


    /*
     * Check whether the awakened task needs to invoke the do_schedule
     * routine. Try to avoid unnecessary runs but:
     * Save approximation: Always switch to scheduler!
     */
    ASSERT(d->processor >= 0);
    ASSERT(d->processor < nr_cpu_ids);
    ASSERT(per_cpu(schedule_data, d->processor).curr);

    //FIXME: Why does calling the scheduler have a lot of overhead, e.g. milliseconds
    // If this check is not done, and we call the scheduler everytime, we will
    // suffer a lot of overhead
    //if ( is_idle_vcpu(current) )
    //if( is_idle_vcpu(per_cpu(schedule_data, d->processor).curr) || EDOM_INFO(per_cpu(schedule_data, d->processor).curr)->local_cputime < 0)

    if( is_idle_vcpu(per_cpu(schedule_data, d->processor).curr) || (inf->status & SC_INACTIVE) || inf->status & SC_MIGRATING ||
        (EDOM_INFO(per_cpu(schedule_data, d->processor).curr)->local_cputime < 0 && inf->local_cputime > 0) )
    {
	DPRINTK3(" -- Calling schedule() --\n");
/*	if(inf->vcpu->domain->domain_id != 0)
	{
	    si = (struct shared_info *) inf->vcpu->domain->shared_info;
	    si->extra_arg7[1] = 2;
	}
*/	cpu_raise_softirq(d->processor, SCHEDULE_SOFTIRQ);
    }
    else
	DPRINTK3(" -- Not calling schedule() --\n");
}

/* Dumps all domains on the specified cpu */
static void sc_dump_cpu_state(const struct scheduler *ops, int i)
{
    DPRINTK("------ CPU: %d - %s ------\n",
	    smp_processor_id(),
	    __func__);

    printk("now=%"PRIu64"\n",NOW());
}



/* Set or fetch domain scheduling parameters */
static int sc_adjust(const struct scheduler *ops, struct domain *p, struct xen_domctl_scheduler_op *op)
{
    //spinlock_t *lock;
    struct shared_info *si;
    struct sc_priv_info *prv = SC_PRIV(ops);
    unsigned long flags;
    s_time_t              now = NOW();
    struct vcpu *v;
    int rc = 0;

    DPRINTK("------ CPU: %d - %s ------\n",
	    smp_processor_id(),
	    __func__);

    printk("--- %s -- now: %llu - domain_id: %d - period: %ld - slice: %ld - vcpu_id: %d - weight: %d ---\n",
	    __func__,
	    (long long unsigned int) now,
	    p->domain_id,
	    op->u.sc.period,
	    op->u.sc.slice,
	    op->u.sc.extratime,
	    op->u.sc.weight);

    if((op->u.sc.period == 2*PERIOD_MAX))
    {
	//Do nothing; don't print; don't collect
	if(sc_debugging == 4)
	{
	    sc_debugging = 1; //Start collecting
	    printk("- Started collecting-\n");
	}
	else if(sc_debugging == 1 || sc_debugging == 3)
	{
	    sc_debugging = 0; //Stop collecting and print
	    printk("- Printing -\n");
	}

	return rc;
    }

    /*
     * Serialize against the pluggable scheduler lock to protect from
     * concurrent updates. We need to take the runq lock for the VCPUs
     * as well, since we are touching slice and
     * period. As in sched_credit2.c, runq locks nest inside the
     * pluggable scheduler lock.
     */
    spin_lock_irqsave(&prv->lock, flags);



    if ( op->cmd == XEN_DOMCTL_SCHEDOP_putinfo )
    {
	si = (struct shared_info *) p->shared_info;

	if(si->extra_arg2[0] == 3)
	{
	    if(EDOM_INFO(p->vcpu[0])->status & SC_RUNNING)
		si->extra_arg2[0] = EDOM_INFO(p->vcpu[0])->cputime + (now-EDOM_INFO(p->vcpu[0])->sched_start_abs);
	    else
		si->extra_arg2[0] = EDOM_INFO(p->vcpu[0])->cputime;

	    goto out;
	}
	else if(si->extra_arg2[1] == 3)
	{
	    if(EDOM_INFO(p->vcpu[1])->status & SC_RUNNING)
		si->extra_arg2[1] = EDOM_INFO(p->vcpu[1])->cputime + (now-EDOM_INFO(p->vcpu[1])->sched_start_abs);
	    else
		si->extra_arg2[1] = EDOM_INFO(p->vcpu[1])->cputime;

	    goto out;
	}

	/* Check for sane parameters */
	if ( !op->u.sc.period )
	{
	    printk("------ cpu: %d - %s - %d ------\n",
		    smp_processor_id(),
		    __func__,
		    __LINE__);

	    rc = -EINVAL;
	    goto out;
	}

	if ( (op->u.sc.period > PERIOD_MAX) ||
		(op->u.sc.period < PERIOD_MIN) ||
		(op->u.sc.slice  > op->u.sc.period) ||
		(op->u.sc.slice  < SLICE_MIN) )
	{
	    printk("------ cpu: %d - %s - %d ------\n",
		    smp_processor_id(),
		    __func__,
		    __LINE__);


	    rc = -EINVAL;
	    goto out;
	}

	/* Time-driven domains */
	for_each_vcpu ( p, v )
	{
	    //if(op->u.sc.weight == 1 && v->vcpu_id != op->u.sc.extratime)
	    if(v->vcpu_id != op->u.sc.extratime)
		continue;

	    op->u.sc.extratime = 0;
	    //--> lock = vcpu_schedule_lock(v);

/*	    if(!dp_wrap_assign_pcpu(v))
	    {
		vcpu_schedule_unlock(lock, v);
		continue;
	    }
*/
	    EDOM_INFO(v)->weight = 0;
	    EDOM_INFO(v)->extraweight = 0;

	    EDOM_INFO(v)->period_temp = op->u.sc.period / 1000;
	    EDOM_INFO(v)->slice_temp  = op->u.sc.slice  / 1000;
/*
	    EDOM_INFO(v)->period_new = EDOM_INFO(v)->period_temp;
	    EDOM_INFO(v)->slice_new  = EDOM_INFO(v)->slice_temp;

	    EDOM_INFO(v)->slice_new = (100000 * EDOM_INFO(v)->slice_new) / EDOM_INFO(v)->period_new;
	    EDOM_INFO(v)->period_new = 100000;
*/
	    printk("-- Before dp-wrap --\n");
	    if(EDOM_INFO(v)->status & SC_DEFAULT)
	    {
		EDOM_INFO(v)->period_new = EDOM_INFO(v)->period_temp;
		EDOM_INFO(v)->slice_new  = EDOM_INFO(v)->slice_temp;

		EDOM_INFO(v)->slice_new = (100000 * EDOM_INFO(v)->slice_new) / EDOM_INFO(v)->period_new;
		EDOM_INFO(v)->period_new = 100000;

		EDOM_INFO(v)->period = op->u.sc.period;
		EDOM_INFO(v)->slice = op->u.sc.slice;
		EDOM_INFO(v)->status &= ~SC_DEFAULT;
	    }
	    else
		tell_vcpus_to_find_new_pcpus(v, &prv->cpu_barrier, ops);
	    printk("-- After dp-wrap --\n");

	    //--> vcpu_schedule_unlock(lock, v);

	}
    }
    else if ( op->cmd == XEN_DOMCTL_SCHEDOP_getinfo )
    {
	if ( p->vcpu[0] == NULL )
	{
	    rc = -EINVAL;
	    goto out;
	}

	op->u.sc.period    = EDOM_INFO(p->vcpu[0])->period;
	op->u.sc.slice     = EDOM_INFO(p->vcpu[0])->slice;
	op->u.sc.extratime = EDOM_INFO(p->vcpu[0])->extratime;
	op->u.sc.latency   = EDOM_INFO(p->vcpu[0])->latency;
	op->u.sc.weight	   = EDOM_INFO(p->vcpu[0])->weight;
    }

out:
    spin_unlock_irqrestore(&prv->lock, flags);

    printk("--- rc value: %d ---\n", rc);
    return rc;
}

/*
static void sc_context_saved(const struct scheduler *ops, struct vcpu *vc)
{
    //struct sc_priv_info *prv = SC_PRIV(ops);
    struct sc_vcpu_info* inf = EDOM_INFO(vc);

    if ( unlikely(is_idle_vcpu(vc)))
	return;

    DPRINTK3("------ CPU: %d - %d - %s - VM: %d.%d ------\n",
	    smp_processor_id(),
	    vc->processor,
	    __func__,
	    vc->domain->domain_id,
	    vc->vcpu_id);

    inf->status &= ~SC_RUNNING;

   if(!(prv->status & SC_CPU0_BUSY) &&
	    vc->processor != smp_processor_id() &&
	    is_idle_vcpu(per_cpu(schedule_data, inf->vcpu->processor).curr))
	cpu_raise_softirq(inf->vcpu->processor, SCHEDULE_SOFTIRQ);
}
*/

static struct sc_priv_info _sc_priv;

const struct scheduler sched_sc_def = {
    .name           = "DP-Wrap",
    .opt_name       = "sc",
    .sched_id       = XEN_SCHEDULER_SC,
    .sched_data     = &_sc_priv,

    .init_domain    = sc_init_domain,
    .destroy_domain = sc_destroy_domain,

    .insert_vcpu    = sc_insert_vcpu,
    .remove_vcpu    = sc_remove_vcpu,
    .alloc_vdata    = sc_alloc_vdata,
    .free_vdata     = sc_free_vdata,
    .alloc_pdata    = sc_alloc_pdata,
    .free_pdata     = sc_free_pdata,
    .alloc_domdata  = sc_alloc_domdata,
    .free_domdata   = sc_free_domdata,

    .init           = sc_init,
    .deinit         = sc_deinit,

    .do_schedule    = sc_do_schedule,
    .pick_cpu       = sc_pick_cpu,
    .dump_cpu_state = sc_dump_cpu_state,
    .sleep          = sc_sleep,
    .wake           = sc_wake,
    .adjust         = sc_adjust,
    //.context_saved  = sc_context_saved,
};

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
