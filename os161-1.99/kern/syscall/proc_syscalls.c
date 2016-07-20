#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>
#include <array.h>
#include <synch.h>
#include <mips/trapframe.h>
#include <kern/limits.h>
#include <kern/fcntl.h>
#include <vfs.h>
#include <limits.h>
#include "opt-A2.h" 



  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */
#if OPT_A2



int sys_fork(struct trapframe *tf, pid_t *retval) {
  struct proc* p = curproc;
  struct proc* child;
  struct addrspace* child_as;
  int err_msg;
  struct semaphore *tf_sem = sem_create("tf_sem",1);
  child = proc_create_runprogram("child");
  if (child == NULL) { return (ENOMEM);}
  err_msg = as_copy(p->p_addrspace, &child_as);
  if (err_msg == ENOMEM) {
    proc_destroy(child);
    return err_msg;
  }
  child->p_addrspace = child_as;

  child->p_pid = generator(child, err_msg);
  if (child->p_pid == NULL) {
    return err_msg;
  }
  child->p_pid->parent = p->p_pid;
  p->p_pid->children++;


  P(tf_sem);
  struct trapframe* ctf = (struct trapframe*)kmalloc(sizeof(struct trapframe));
  *ctf = *tf; // tf points to parent's trapframe;
  err_msg = thread_fork("child_thread", child, (void*)enter_forked_process, 
    (void*)ctf, (int)child->p_pid->cpid);
  V(tf_sem);
  if (err_msg == ENOMEM) {
    lock_acquire(recycle_lk);
    array_add(recycle_array, &child->p_pid->cpid, (void*)recycle);
    lock_release(recycle_lk);
    lock_acquire(running_proc_lk);
    for (unsigned i = 0; i < array_num(running_proc); i++) {
        if (child->p_pid == (struct pid *) array_get(running_proc, i)) {
           array_remove(running_proc, i);
        }
      }
    lock_release(running_proc_lk);
    kfree(child->p_pid);
    proc_destroy(child);
    return err_msg;
  }
  *retval = child->p_pid->cpid;
  return 0;
}


#endif /* OPT_A2 */

void sys__exit(int exitcode) {

  struct addrspace *as;
  struct proc* p = curproc;
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */

#if OPT_A2
  
  lock_acquire(running_proc_lk);
  struct pid* p_pid = p->p_pid; 
  if (p_pid->children != 0) {
    if (p_pid->running) p_pid->running = false;
    unsigned proc_num = array_num(running_proc);
    for (unsigned i = 0; i < proc_num ; ++i) {
      struct pid* temp = (struct pid*)array_get(running_proc, i);

      if (temp->parent == p_pid) { // if its children's pid still in use
        if (!temp->running) { // if the children is not running
          array_remove(running_proc, i);
          lock_acquire(recycle_lk);
          array_add(recycle_array, &temp->cpid, (void*)recycle);
          lock_release(recycle_lk);
          kfree(temp);
          p_pid->children--;
          --proc_num;
          --i;
        } else { // if the children is still running
          
          temp->parent = NULL;
        } 
      }
    }
  } 

  if (p_pid->children == 0) { // if this pid doesn't have any children, or its children are died
    if (p_pid->running) p_pid->running = false;
    if (p_pid->parent == NULL) { // if pid has no parent, no child
      for (unsigned i = 0; i < array_num(running_proc); i++) {
        if (p_pid == (struct pid *) array_get(running_proc, i)) {
           array_remove(running_proc, i);
           break;
        }
      }
      lock_acquire(recycle_lk);
      array_add(recycle_array, &p_pid->cpid, (void*)recycle);
      lock_release(recycle_lk);
      kfree(p_pid);

    } else {  // have parent, no children
      p_pid->parent->exitcode = _MKWAIT_EXIT(exitcode);
      cv_broadcast(runprogram_cv, running_proc_lk);
    }
  }
  lock_release(running_proc_lk);
  #endif /* OPT_A2 */



  KASSERT(curproc->p_addrspace != NULL);
  as_deactivate();
  /*
   * clear p_addrspace before calling as_destroy. Otherwise if
   * as_destroy sleeps (which is quite possible) when we
   * come back we'll be calling as_activate on a
   * half-destroyed address space. This tends to be
   * messily fatal.
   */
  as = curproc_setas(NULL);
  as_destroy(as);

  /* detach this thread from its process */
  /* note: curproc cannot be used after this call */
  proc_remthread(curthread);

  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
  proc_destroy(p);
  
  thread_exit();
  /* thread_exit() does not return, so we should never get here */
  panic("return from thread_exit in sys_exit\n");
}


/* stub handler for getpid() system call               */

int
sys_getpid(pid_t *retval)
{
  /* for now, this is just a stub that always returns a PID of 1 */
  /* you need to fix this to make it work properly */
#if OPT_A2
  *retval = curproc->p_pid->cpid;
#else
  *retval = 1;
#endif

  return (0);
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid,
	    userptr_t status,
	    int options,
	    pid_t *retval)
{
  int exitstatus;
  int result;

  /* this is just a stub implementation that always reports an
     exit status of 0, regardless of the actual exit status of
     the specified process.   
     In fact, this will return 0 even if the specified process
     is still running, and even if it never existed in the first place.

     Fix this!
  */
#if OPT_A2
     struct proc* p = curproc;
     lock_acquire(running_proc_lk);
     unsigned num_proc = array_num(running_proc);
     bool found = false;
     for (unsigned i = 0; i < num_proc; i++) {
      struct pid* temp = (struct pid*) array_get(running_proc, i);
      if (temp->cpid != pid) continue;
      if (temp->cpid == pid && temp->parent != p->p_pid) {
        lock_release(running_proc_lk);
        return (ECHILD);
      } else {
        while (temp->running) {
          cv_wait(runprogram_cv, running_proc_lk);
        }
        if (!temp->running) {
          exitstatus = p->p_pid->exitcode;
        }
      }
      found = true;
      break;
     }
     lock_release(running_proc_lk);
     if (!found) {
      return (ESRCH);
     }

#endif /*OPT_A2 */

  if (options != 0) {
    return(EINVAL);
  }
  /* for now, just pretend the exitstatus is 0 */
  //exitstatus = 0;
  result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    return(result);
  }
  //DEBUG(DB_SYSCALL, "pid %d \n", pid);
  *retval = pid;
  return(0);
}

#if OPT_A2

int sys_execv(char* progname, char** args) {
  struct addrspace* as;
  struct vnode *v;
  int result;
  vaddr_t entrypoint, stackptr;
  struct addrspace* old_as;
  size_t stoplen;

  DEBUG(DB_SYSCALL, "enter sys_execv\n");
  //check if the args is a vaild user space
  char** argg;
  result = copyin((const_userptr_t) args, (void*)&argg, sizeof(char**));
  if (result) {
    return result;
  }
  DEBUG(DB_SYSCALL, "after check\n");

  //count the # of argrements
  int argc = 0;
  while (1) {
    if (args[argc] == NULL) break;
    argc++;
  }
  argc++; // the NULL terminate
  if (argc > ARG_MAX) {
    return E2BIG;
  }

  //copy these arguements into kernel
  char** kargs = kmalloc(sizeof(char*) * argc);
  for (int i = 0; i < argc - 1; i++) {
    int arglen = strlen(args[i]) + 1;
    kargs[i] = kmalloc(sizeof(char) * arglen);
    result = copyinstr((const_userptr_t) args[i], kargs[i], sizeof(char) * arglen, &stoplen);
    if (result) {
      for (int j = 0; j < i; j++) {
        kfree(kargs[j]);
      }
      kfree(kargs);
      return result;
    }
  }
  kargs[argc-1] = NULL;


  DEBUG(DB_SYSCALL, "before copy path\n");
  //copy the program path into kernel
  int proglen = (strlen(progname) + 1);
  char* kprogname = kmalloc(sizeof(char) * proglen);
  result = copyinstr((const_userptr_t)progname, kprogname, proglen, &stoplen);
  if (result) {
    for (int j = 0; j < argc - 1; j++) {
        kfree(kargs[j]);
      }
      kfree(kargs);
      kfree(progname);
      return result;
  }


  result = vfs_open(kprogname, O_RDONLY, 0, &v);
  if (result) {
    for(int i = 0; i < argc - 1; i++) {
      kfree(kargs[i]);
    }
    kfree(kargs);
    kfree(kprogname);
    return result;
  }

  DEBUG(DB_SYSCALL, "before as create\n");
  /* Create a new address space. */
  as = as_create();
  if (as == NULL) {
    for (int j = 0; j < argc - 1; j++) {
        kfree(kargs[j]);
      }
    kfree(kargs);
    kfree(kprogname);
    vfs_close(v);
    return ENOMEM;
  }

  /* Switch to it and activate it. */
  old_as = curproc_setas(as);
  as_activate();

  /* Load the executable. */
  result = load_elf(v, &entrypoint);
  if (result) {
    vfs_close(v);
    for (int j = 0; j < argc-1; j++) {
        kfree(kargs[j]);
      }
    kfree(kargs);
    kfree(kprogname);
    return result;
  }
  /* Done with the file now. */
  vfs_close(v);

  /* Define the user stack in the address space */
  result = as_define_stack(as, &stackptr);
  if (result) {
    for (int j = 0; j < argc-1; j++) {
        kfree(kargs[j]);
      }
    kfree(kargs);
    kfree(kprogname);
    return result;
  }

  userptr_t *user_args = kmalloc(sizeof(user_args) * argc);
  for (int i = 0; i < argc -1; i++) {
    int len = strlen(kargs[i]) + 1;
    len = ROUNDUP(len, 8);
    stackptr -= len;
    user_args[i] = (userptr_t)stackptr; 
    result = copyoutstr(kargs[i], (userptr_t) stackptr, len, &stoplen);
    if (result) {
      return result;
    }
  }
  user_args[argc-1] = NULL;

  int argc_len = ROUNDUP((argc * sizeof(char*)), 8);
  stackptr -= argc_len;
  DEBUG(DB_SYSCALL, "before last copycout\n");
  result = copyout(user_args, (userptr_t)stackptr, (size_t)argc_len);
  if (result) {
    return result;
  }
  kfree(user_args);

  //delete old address space
  as_destroy(old_as);

  /*Call enter_new_process with address to the arguments on the stack, 
  the stack pointer (from as_define_stack), 
  and the program entry point (from vfs_open)*/
  enter_new_process(argc-1 , (userptr_t)stackptr, stackptr, entrypoint);

  return 0;


}

#endif //OPT_A2









