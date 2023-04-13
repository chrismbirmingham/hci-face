import Head from 'next/head';
// import styles from '../styles/Home.module.css';
// import Link from 'next/link';

export default function Home() {
  return (
    <div className='text-6xl text-green-600 p-2'>
      <Head>
        <title>Create Next App</title>
        <link rel="icon" href="/favicon.ico" />
      </Head>

      <main className='text-6xl text-green-600 p-2'>
        <div className='text-4xl text-green-600 p-2'>
          Welcome to HCI Face!
        </div>
        <h3 className="title">
            Pages:
        </h3>
        <br></br>Make your own transcription on <a href="/transcription">this page!</a>
        <br></br>Puppet the face with <a href="/puppeteering">this page!</a>
        <br></br>Have a text conversation on <a href="/text_conversation">this page!</a>
        <br></br>Interact with a robot on <a href="/multimodal_interaction">this page!</a>
        <br></br>
        <br></br>Show just the face on <a href="/face">this page!</a>
        <br></br>Play with Action units on <a href="/playfaces">this page!</a>
        <br></br>Fine tune your mic with <a href="/mic_playground">this page!</a>

      </main>

      {/* <style jsx>{`
        main {
          padding: 5rem 0;
          flex: 1;
          display: flex;
          flex-direction: column;
          justify-content: center;
          align-items: center;
        }
        footer {
          width: 100%;
          height: 100px;
          border-top: 1px solid #eaeaea;
          display: flex;
          justify-content: center;
          align-items: center;
        }
        footer img {
          margin-left: 0.5rem;
        }
        footer a {
          display: flex;
          justify-content: center;
          align-items: center;
          text-decoration: none;
          color: inherit;
        }
        code {
          background: #fafafa;
          border-radius: 5px;
          padding: 0.75rem;
          font-size: 1.1rem;
          font-family: Menlo, Monaco, Lucida Console, Liberation Mono,
            DejaVu Sans Mono, Bitstream Vera Sans Mono, Courier New, monospace;
        }
      `}</style>

      <style jsx global>{`
        html,
        body {
          padding: 0;
          margin: 0;
          font-family: -apple-system, BlinkMacSystemFont, Segoe UI, Roboto,
            Oxygen, Ubuntu, Cantarell, Fira Sans, Droid Sans, Helvetica Neue,
            sans-serif;
        }
        * {
          box-sizing: border-box;
        }
      `}</style> */}
    </div>
  )
}
