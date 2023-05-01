import Head from 'next/head';
import Chatbot from '../public/Chatbot.svg'
import TalkingRobot from '../public/TalkingRobot.svg'
import Puppetshow from '../public/Puppetshow.svg'
import Recording from '../public/Recording-pana.svg'
import Image from 'next/image';


export default function Home() {
  return (
    <div className='Front Page'>
      <Head>
        <title>HCI FACE</title>
        <link rel="icon" href="/favicon.ico" />
      </Head>

      <main className='text-2xl p-2'>
        <div className='text-4xl p-8'>
          Welcome to HCI Face!
        </div>

        <div id="fullWidthTabContent" className="border-t border-gray-200 dark:border-gray-600">
          <div className="p-4 bg-white rounded-lg md:p-8 dark:bg-gray-800" id="stats" role="tabpanel" aria-labelledby="stats-tab">
              <dl className="grid max-w-screen-xl grid-cols-2 gap-8 p-4 mx-auto text-gray-900 sm:grid-cols-3 xl:grid-cols-4 dark:text-white sm:p-8">
                  
                <div className="max-w-sm p-6 bg-white border border-gray-200 rounded-lg shadow dark:bg-gray-800 dark:border-gray-700">
                  <a href="/transcription">
                    <Image priority src={Recording} alt='next.js logo'/>
                      <h5 className="mb-2 text-2xl font-bold tracking-tight text-gray-900 dark:text-white">Record your ideas</h5>
                  </a>
                  <p className="mb-3 font-normal text-gray-700 dark:text-gray-400">Freely record ideas. You can either actively control a microphone or have it passively take notes on your next idea session.</p>
                </div>
                <div className="max-w-sm p-6 bg-white border border-gray-200 rounded-lg shadow dark:bg-gray-800 dark:border-gray-700">
                  <a href="/text_conversation">
                    <Image priority src={Chatbot} alt='next.js logo'/>
                      <h5 className="mb-2 text-2xl font-bold tracking-tight text-gray-900 dark:text-white">Text with a LLM</h5>
                  </a>
                  <p className="mb-3 font-normal text-gray-700 dark:text-gray-400">Have a text-based conversation with a LLM. You get to describe what kind of bot you want to talk to!</p>
                </div>
                <div className="max-w-sm p-6 bg-white border border-gray-200 rounded-lg shadow dark:bg-gray-800 dark:border-gray-700">
                  <a href="/puppeteering">
                    <Image priority src={Puppetshow} alt='next.js logo'/>
                      <h5 className="mb-2 text-2xl font-bold tracking-tight text-gray-900 dark:text-white">Puppeteer the robot Face</h5>
                  </a>
                  <p className="mb-3 font-normal text-gray-700 dark:text-gray-400">Test out the behaviors, expressions and speech capabilities of the robot face.</p>
                </div>
                <div className="max-w-sm p-6 bg-white border border-gray-200 rounded-lg shadow dark:bg-gray-800 dark:border-gray-700">
                  <a href="/multimodal_interaction">
                    <Image priority src={TalkingRobot} alt='next.js logo'/>
                      <h5 className="mb-2 text-2xl font-bold tracking-tight text-gray-900 dark:text-white">Have a conversation</h5>
                  </a>
                  <p className="mb-3 font-normal text-gray-700 dark:text-gray-400">Have a voice based conversation with a chatbot. You get to describe what kind of bot you want to talk to!</p>
                </div>
              </dl>
      <a href="https://storyset.com/happy">Happy illustrations by Storyset</a>
          </div>
        </div>
        <h3 className="text-3xl">
            Additional Pages:
        </h3>
        <br></br>Show just the face on <a className="text-blue-600" href="/face">this page!</a>
        <br></br>Play with Action units on <a className="text-blue-600" href="/playfaces">this page!</a>
        <br></br>Fine tune your mic settings with <a className="text-blue-600" href="/mic_playground">this page!</a>
        <br></br>Use the clock on <a className="text-blue-600" href="/clock">this page!</a>
      </main>
    </div>
  )
}
