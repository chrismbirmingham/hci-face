

export default function getDeadTime (minutes, seconds=0) {
    console.log("New goal: ", minutes)
    let deadline = new Date();
    let newseconds = minutes*60 + 1
    deadline.setSeconds(deadline.getSeconds() + newseconds);
    return deadline;
  }